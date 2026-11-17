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

const SCENARIO = "frozen-motor-encoder";
const FIRST_TICK = 640;
const LAST_TICK = 1240;
const TICKS_PER_FRAME = 8;
const FRAMES_PER_SECOND = 20; // 8 ms of simulation per 50 ms of GIF: slowed about 6 times.
const VIEWPORT = { width: 1180, height: 1010 };

const [baseUrl, outputDirectory] = process.argv.slice(2);
if (!baseUrl || !outputDirectory) {
  console.error("usage: capture_viewer.mjs <viewer url> <output directory>");
  process.exit(2);
}
mkdirSync(outputDirectory, { recursive: true });
const frames = mkdtempSync(join(tmpdir(), "stapes-frames-"));

// Software rendering, so the capture does not depend on the machine's graphics driver.
const browser = await chromium.launch({
  args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
});
const page = await browser.newPage({ viewport: VIEWPORT });
page.on("pageerror", (error) => {
  console.error(`page error: ${error.message}`);
  process.exitCode = 1;
});
await page.goto(`${baseUrl}/index.html#${SCENARIO}`);
await page.waitForFunction(() => document.getElementById("scrub").max !== "1");

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

let index = 0;
for (let tick = FIRST_TICK; tick <= LAST_TICK; tick += TICKS_PER_FRAME) {
  await showTick(tick);
  await page.screenshot({ path: join(frames, `${String(index).padStart(4, "0")}.png`) });
  index += 1;
}
await showTick(1100);
await page.screenshot({ path: join(outputDirectory, "viewer.png") });
await browser.close();

// One palette for the whole GIF, computed from its own frames, keeps the colours steady.
execFileSync(
  "ffmpeg",
  [
    "-y", "-loglevel", "error", "-framerate", String(FRAMES_PER_SECOND),
    "-i", join(frames, "%04d.png"),
    "-vf", "scale=880:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=none",
    join(outputDirectory, "viewer.gif"),
  ],
  { stdio: "inherit" },
);
rmSync(frames, { recursive: true });
console.log(`${index} frames written to ${join(outputDirectory, "viewer.gif")}`);
