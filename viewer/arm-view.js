// The 3D view: the arm as a chain of links, and the planned tunnel.
//
// It draws positions that the simulation already computed. There is no kinematics here, so the
// picture cannot disagree with the numbers.

import * as THREE from "three";
import { OrbitControls } from "./vendor/OrbitControls.js";

const INK = 0x0b1f2a;
const TEAL = 0x0f766e;
const PALE_TEAL = 0xccfbf1;
const SIGNAL_RED = 0xd92d20;

const LINK_RADIUS_M = [0.034, 0.03, 0.026, 0.02, 0.017, 0.006];
const UP = new THREE.Vector3(0, 1, 0);

export class ArmView {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    this.scene = new THREE.Scene();
    // The simulation's world has z up, as robotics does. three.js defaults to y up.
    this.camera = new THREE.PerspectiveCamera(35, 1, 0.01, 20);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(0.95, -0.75, 0.75);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0.1, 0.05, 0.35);
    this.controls.enableDamping = true;
    this.controls.addEventListener("change", () => this.requestRender());

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xb8c9c7, 2.2));
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(1.5, -1.0, 2.5);
    this.scene.add(key);

    this.#buildFloor();
    this.#buildArm();
    this.tunnel = new THREE.Group();
    this.scene.add(this.tunnel);

    this.needsRender = true;
    new ResizeObserver(() => this.#resize()).observe(canvas);
    this.#resize();
    const loop = () => {
      this.controls.update();
      if (this.needsRender) {
        this.renderer.render(this.scene, this.camera);
        this.needsRender = false;
      }
      requestAnimationFrame(loop);
    };
    loop();
  }

  requestRender() {
    this.needsRender = true;
  }

  #resize() {
    const { clientWidth, clientHeight } = this.canvas.parentElement;
    this.renderer.setSize(clientWidth, clientHeight, false);
    this.camera.aspect = clientWidth / Math.max(clientHeight, 1);
    this.camera.updateProjectionMatrix();
    this.requestRender();
  }

  #buildFloor() {
    const grid = new THREE.GridHelper(2, 20, 0x9fb3b0, 0xd5dee1);
    grid.rotation.x = Math.PI / 2; // GridHelper lies in the x-z plane; the floor here is x-y.
    this.scene.add(grid);
  }

  #buildArm() {
    const linkMaterial = new THREE.MeshStandardMaterial({ color: 0xe8eef0, roughness: 0.55 });
    const jointMaterial = new THREE.MeshStandardMaterial({ color: TEAL, roughness: 0.4 });
    this.toolMaterial = new THREE.MeshStandardMaterial({ color: INK, roughness: 0.3 });

    // The pedestal from the floor to the first joint. It never moves.
    this.pedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.06, 1, 32), linkMaterial);
    this.scene.add(this.pedestal);

    this.links = LINK_RADIUS_M.map((radius, index) => {
      const isTool = index === LINK_RADIUS_M.length - 1;
      const mesh = new THREE.Mesh(
        new THREE.CylinderGeometry(radius, radius, 1, 24),
        isTool ? this.toolMaterial : linkMaterial,
      );
      this.scene.add(mesh);
      return mesh;
    });
    this.joints = LINK_RADIUS_M.map((radius) => {
      const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius * 1.25, 24, 16), jointMaterial);
      this.scene.add(mesh);
      return mesh;
    });
  }

  // Stretches a unit cylinder so that it runs from `from` to `to`.
  #span(mesh, from, to) {
    const along = new THREE.Vector3().subVectors(to, from);
    const length = along.length();
    mesh.visible = length > 1e-6;
    if (!mesh.visible) {
      return;
    }
    mesh.position.copy(from).addScaledVector(along, 0.5);
    mesh.scale.set(1, length, 1);
    mesh.quaternion.setFromUnitVectors(UP, along.divideScalar(length));
  }

  setTunnel(entry, target) {
    this.tunnel.clear();
    const from = new THREE.Vector3(...entry);
    const to = new THREE.Vector3(...target);
    const direction = new THREE.Vector3().subVectors(to, from).normalize();

    // An illustrative block of bone around the tunnel. Its size is chosen to be visible; only
    // the tunnel's position and direction come from the plan.
    const bone = new THREE.Mesh(
      new THREE.BoxGeometry(0.07, 0.07, 0.03),
      new THREE.MeshStandardMaterial({
        color: PALE_TEAL,
        transparent: true,
        opacity: 0.55,
        roughness: 0.9,
      }),
    );
    bone.position.copy(from).addScaledVector(direction, 0.015);
    bone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), direction);
    this.tunnel.add(bone);

    // The planned line, extended back out of the bone so its direction can be seen.
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([
        from.clone().addScaledVector(direction, -0.12),
        to.clone(),
      ]),
      new THREE.LineDashedMaterial({ color: TEAL, dashSize: 0.008, gapSize: 0.005 }),
    );
    line.computeLineDistances();
    this.tunnel.add(line);
    this.requestRender();
  }

  // `chain` is seven points: the six joint origins and the tool tip.
  setPose(chain, stopped) {
    const points = chain.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
    this.#span(this.pedestal, new THREE.Vector3(points[0].x, points[0].y, 0), points[0]);
    for (let i = 0; i < this.links.length; i += 1) {
      this.#span(this.links[i], points[i], points[i + 1]);
      this.joints[i].position.copy(points[i]);
    }
    // The tool turns red when, and only when, the monitor has stopped the robot.
    this.toolMaterial.color.setHex(stopped ? SIGNAL_RED : INK);
    this.requestRender();
  }
}
