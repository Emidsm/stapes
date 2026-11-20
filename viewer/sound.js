// Sound for the replay, synthesised with Web Audio: no audio files.
//
// Like everything else on the page, it only reads what the recording already holds. The motor
// hum follows how fast the arm is moving, the drill whine plays while the tip advances through
// bone, and the alarm sounds at the safety stop, the one event that is also drawn in red.
//
// It is off until the viewer turns it on, which is also what browsers require before a page
// may play sound.

const HUM_HZ = 70;
const WHINE_HZ = 1650;
const ALARM_HIGH_HZ = 880;
const ALARM_LOW_HZ = 660;
// Arm speed, in m/s summed over the joints, that gives the full hum.
const FULL_HUM_M_S = 0.05;
const RAMP_S = 0.05;

export class SoundBoard {
  constructor() {
    this.context = null;
    this.enabled = false;
    this.alarmedAtTick = null;
  }

  // Must be called from a user gesture the first time.
  async setEnabled(enabled) {
    this.enabled = enabled;
    if (enabled && this.context === null) {
      this.#build();
    }
    if (this.context !== null) {
      if (enabled) {
        await this.context.resume();
      } else {
        this.silence();
        await this.context.suspend();
      }
    }
  }

  #build() {
    const context = new AudioContext();
    this.context = context;

    // Motor hum: a low sawtooth, filtered so it reads as a motor and not as a buzzer.
    const hum = context.createOscillator();
    hum.type = "sawtooth";
    hum.frequency.value = HUM_HZ;
    const humFilter = context.createBiquadFilter();
    humFilter.type = "lowpass";
    humFilter.frequency.value = 400;
    this.humGain = context.createGain();
    this.humGain.gain.value = 0;
    hum.connect(humFilter).connect(this.humGain).connect(context.destination);
    hum.start();

    // Drill whine: a high triangle with a slow wobble, as a burr under load.
    const whine = context.createOscillator();
    whine.type = "triangle";
    whine.frequency.value = WHINE_HZ;
    const wobble = context.createOscillator();
    wobble.frequency.value = 9;
    const wobbleDepth = context.createGain();
    wobbleDepth.gain.value = 25;
    wobble.connect(wobbleDepth).connect(whine.frequency);
    this.whineGain = context.createGain();
    this.whineGain.gain.value = 0;
    whine.connect(this.whineGain).connect(context.destination);
    whine.start();
    wobble.start();
  }

  // Sets the continuous sounds for the moment being shown. `armSpeed` is in m/s, summed over the
  // joints; `drilling` is whether the tip is advancing through bone.
  update(armSpeed, drilling) {
    if (!this.enabled || this.context === null) {
      return;
    }
    const now = this.context.currentTime;
    const hum = Math.min(1, armSpeed / FULL_HUM_M_S) * 0.12;
    this.humGain.gain.setTargetAtTime(hum, now, RAMP_S);
    this.whineGain.gain.setTargetAtTime(drilling ? 0.05 : 0, now, RAMP_S);
  }

  silence() {
    if (this.context === null) {
      return;
    }
    const now = this.context.currentTime;
    this.humGain.gain.setTargetAtTime(0, now, RAMP_S);
    this.whineGain.gain.setTargetAtTime(0, now, RAMP_S);
  }

  // Plays the alarm once each time playback reaches the stop. Scrubbing back before the stop
  // re-arms it.
  passStop(stopTick, tick) {
    if (stopTick === null || tick < stopTick) {
      this.alarmedAtTick = null;
      return;
    }
    if (this.alarmedAtTick === stopTick) {
      return;
    }
    this.alarmedAtTick = stopTick;
    if (!this.enabled || this.context === null) {
      return;
    }
    const context = this.context;
    const start = context.currentTime;
    for (let beep = 0; beep < 3; beep += 1) {
      const tone = context.createOscillator();
      tone.type = "square";
      tone.frequency.value = beep % 2 === 0 ? ALARM_HIGH_HZ : ALARM_LOW_HZ;
      const gain = context.createGain();
      const at = start + beep * 0.22;
      gain.gain.setValueAtTime(0, at);
      gain.gain.linearRampToValueAtTime(0.08, at + 0.01);
      gain.gain.setValueAtTime(0.08, at + 0.17);
      gain.gain.linearRampToValueAtTime(0, at + 0.19);
      tone.connect(gain).connect(context.destination);
      tone.start(at);
      tone.stop(at + 0.2);
    }
  }
}
