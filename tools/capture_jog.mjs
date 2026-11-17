#!/usr/bin/env node
// Captures the hands-on mode as a GIF for the README: joints turned one at a time, and the view.
//
// The hands-on server must be running, with no gamepad in the way of the keyboard:
//
//   python -m stapes jog --browser-gamepad &
//   node tools/capture_jog.mjs http://127.0.0.1:8765 docs/media
//
// Needs Playwright and ffmpeg, as tools/capture_viewer.mjs does.
//
// The page's clock is Playwright's, advanced 50 ms at a time and only once the simulation has
// answered, so the GIF runs at real speed however slow the machine renders. The session the
// capture drove is recorded by the server, like any other, and replays exactly with
// `python -m stapes jog --replay`.

import { execFileSync } from "node:child_process";
import { mkdirSync, mkdtempSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { chromium } from "playwright";

const VIEWPORT = { width: 1180, height: 1010 };
const FRAMES_PER_SECOND = 10;
// The page sends one step per animation frame, of at most 50 ms: one clock advance each.
const STEP_MS = 50;
const STEPS_PER_FRAME = 1000 / FRAMES_PER_SECOND / STEP_MS;

// What the operator does: keys held for a time, or pressed once. [ and ] choose the joint.
const SCRIPT = [
  { press: "KeyJ", note: "joint mode" },
  { wait: 600 },
  { hold: "KeyD", ms: 2600, note: "joint 1, the base" },
  { wait: 900 },
  { press: "BracketRight" },
  { hold: "KeyA", ms: 2200, note: "joint 2, the shoulder" },
  { wait: 900 },
  { press: "BracketRight" },
  { hold: "KeyD", ms: 2200, note: "joint 3, the elbow" },
  { wait: 900 },
  { hold: "ArrowRight", ms: 1500, note: "turn the view" },
  { press: "BracketRight" },
  { press: "BracketRight" },
  { hold: "KeyD", ms: 2200, note: "joint 5, the wrist" },
  { wait: 1200 },
];

const [baseUrl, outputDirectory] = process.argv.slice(2);
if (!baseUrl || !outputDirectory) {
  console.error("usage: capture_jog.mjs <server url> <output directory>");
  process.exit(2);
}
mkdirSync(outputDirectory, { recursive: true });

const browser = await chromium.launch({
  args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
});
const page = await browser.newPage({ viewport: VIEWPORT });
// Installed and paused: the page's time moves only when this script says, never while a slow
// screenshot is being taken.
await page.clock.install();
await page.clock.pauseAt(Date.now() + 1000);
page.on("pageerror", (error) => {
  console.error(`page error: ${error.message}`);
  process.exitCode = 1;
});

const frames = mkdtempSync(join(tmpdir(), "stapes-jog-frames-"));
let index = 0;
let steps = 0;
// Advances the page by one step and waits for the simulation's answer.
async function step() {
  const answered = page.waitForResponse((response) => response.url().endsWith("/api/jog/step"));
  await page.clock.runFor(STEP_MS);
  await answered;
  steps += 1;
  if (steps % STEPS_PER_FRAME === 0) {
    await page.clock.runFor(1); // One more animation frame, to draw what came back.
    await page.screenshot({ path: join(frames, `${String(index).padStart(4, "0")}.png`) });
    index += 1;
  }
}
// Films `ms` of simulated time.
async function film(ms) {
  for (let t = 0; t < ms; t += STEP_MS) {
    await step();
  }
}

await page.goto(`${baseUrl}/jog.html`);
await page.clock.runFor(500);
await page.waitForFunction(() => document.getElementById("mode").textContent === "Free");
await film(800);
for (const action of SCRIPT) {
  if (action.press) {
    await page.keyboard.press(action.press);
    await film(250);
  } else if (action.hold) {
    await page.keyboard.down(action.hold);
    await film(action.ms);
    await page.keyboard.up(action.hold);
  } else {
    await film(action.wait);
  }
}
await browser.close();

execFileSync(
  "ffmpeg",
  [
    "-y", "-loglevel", "error", "-framerate", String(FRAMES_PER_SECOND),
    "-i", join(frames, "%04d.png"),
    "-vf", "scale=880:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=none",
    join(outputDirectory, "joint.gif"),
  ],
  { stdio: "inherit" },
);
rmSync(frames, { recursive: true });
console.log(`${index} frames written to ${join(outputDirectory, "joint.gif")}`);
