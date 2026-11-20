#!/usr/bin/env node
// Captures the viewer as a GIF and a still image, for the README.
//
// The viewer must be served over HTTP with its traces in place:
//
//   python -m stapes export viewer/traces
//   python3 -m http.server 8765 -d viewer &
//   node tools/capture_viewer.mjs http://127.0.0.1:8765 docs/media
//
// Needs Playwright (`npm install playwright`, then `npx playwright install chromium`) and ffmpeg.
// Neither is a dependency of the project: this script is only run to refresh the media.
//
// Frames are taken by moving the viewer's own time slider, one recorded tick at a time, so the
// GIF shows exact ticks of the recording and is the same on a fast or a slow machine.

import { execFileSync } from "node:child_process";
import { mkdirSync, mkdtempSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { chromium } from "playwright";

// Each capture is one GIF. `step` ticks of simulation pass per GIF frame, at 20 frames a second.
const CAPTURES = [
  // A motor encoder freezes while drilling: 8 ms per 50 ms frame, slowed about 6 times.
  { name: "viewer", scenario: "frozen-motor-encoder", first: 640, last: 1240, step: 8, still: 1100 },
  // The whole run from rest, in the wide view: about twice real time.
  { name: "approach", scenario: "approach", first: 0, last: 18690, step: 90 },
  // The last part of the same run, with the camera beside the tool: about real time.
  { name: "drilling", scenario: "approach", first: 14800, last: 18690, step: 50, follow: true },
];
const FRAMES_PER_SECOND = 20;
const VIEWPORT = { width: 1180, height: 1010 };

const [baseUrl, outputDirectory] = process.argv.slice(2);
if (!baseUrl || !outputDirectory) {
  console.error("usage: capture_viewer.mjs <viewer url> <output directory>");
  process.exit(2);
}
mkdirSync(outputDirectory, { recursive: true });

// Software rendering, so the capture does not depend on the machine's graphics driver.
const browser = await chromium.launch({
  args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
});
const page = await browser.newPage({ viewport: VIEWPORT });
page.on("pageerror", (error) => {
  console.error(`page error: ${error.message}`);
  process.exitCode = 1;
});

async function showTick(tick) {
  await page.evaluate((value) => {
    const slider = document.getElementById("scrub");
    slider.value = String(value);
    slider.dispatchEvent(new Event("input"));
  }, tick);
  // Two animation frames: one for the page to draw, one for the 3D view's own render loop.
  await page.evaluate(
    () => new Promise((done) => requestAnimationFrame(() => requestAnimationFrame(done))),
  );
}

async function capture({ name, scenario, first, last, step, still, follow }) {
  const frames = mkdtempSync(join(tmpdir(), "stapes-frames-"));
  await page.goto(`${baseUrl}/index.html#${scenario}`);
  await page.reload();
  await page.waitForFunction(() => document.getElementById("scrub").max !== "1");
  if (follow) {
    await page.click("#follow");
  }
  let index = 0;
  for (let tick = first; tick <= last; tick += step) {
    await showTick(tick);
    await page.screenshot({ path: join(frames, `${String(index).padStart(4, "0")}.png`) });
    index += 1;
  }
  if (still !== undefined) {
    await showTick(still);
    await page.screenshot({ path: join(outputDirectory, `${name}.png`) });
  }

  // One palette for the whole GIF, computed from its own frames, keeps the colours steady.
  execFileSync(
    "ffmpeg",
    [
      "-y", "-loglevel", "error", "-framerate", String(FRAMES_PER_SECOND),
      "-i", join(frames, "%04d.png"),
      "-vf", "scale=880:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=none",
      join(outputDirectory, `${name}.gif`),
    ],
    { stdio: "inherit" },
  );
  rmSync(frames, { recursive: true });
  console.log(`${index} frames written to ${join(outputDirectory, `${name}.gif`)}`);
}

for (const settings of CAPTURES) {
  await capture(settings);
}
await browser.close();
