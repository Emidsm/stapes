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
// The piece the operator has chosen to turn. Amber, never red: red means a stop or a fault.
const CHOSEN = 0xf59e0b;

const LINK_RADIUS_M = [0.034, 0.03, 0.026, 0.02, 0.017, 0.003];
const UP = new THREE.Vector3(0, 1, 0);
// How far the camera sits from the tip when following it: close enough to see the 10 mm feed,
// far enough to see the tool enter the bone.
const FOLLOW_DISTANCE_M = 0.1;

export class ArmView {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    this.scene = new THREE.Scene();
    // The simulation's world has z up, as robotics does. three.js defaults to y up.
    this.camera = new THREE.PerspectiveCamera(35, 1, 0.002, 20);
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

    this.following = false;
    this.tip = null;
    this.wideView = null;

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
    this.jointMaterial = jointMaterial;
    this.linkMaterial = linkMaterial;
    this.chosenMaterial = new THREE.MeshStandardMaterial({
      color: CHOSEN,
      emissive: CHOSEN,
      emissiveIntensity: 0.25,
      roughness: 0.4,
    });
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
    this.tunnelDirection = direction.clone();

    // An illustrative block of bone around the tunnel. Its size is chosen to be visible; only
    // the tunnel's position and direction come from the plan.
    const bone = new THREE.Mesh(
      new THREE.BoxGeometry(0.07, 0.07, 0.03),
      new THREE.MeshStandardMaterial({
        color: PALE_TEAL,
        transparent: true,
        opacity: 0.4,
        depthWrite: false,
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

  // Keeps the camera on the tool tip, close up, or returns it to the wide view. While following,
  // the camera moves with the tip and can still be orbited and zoomed around it.
  setFollowing(following) {
    if (following === this.following || this.tip === null) {
      return;
    }
    this.following = following;
    if (following) {
      this.wideView = {
        position: this.camera.position.clone(),
        target: this.controls.target.clone(),
      };
      // From the side of the tunnel, a little above it, so the feed moves across the picture
      // instead of straight away from the camera.
      const direction = this.tunnelDirection ?? new THREE.Vector3(1, 0, 0);
      const from = new THREE.Vector3().subVectors(this.camera.position, this.controls.target);
      from.addScaledVector(direction, -from.dot(direction));
      if (from.lengthSq() < 1e-9) {
        from.crossVectors(direction, new THREE.Vector3(0, 0, 1));
      }
      from.normalize().addScaledVector(new THREE.Vector3(0, 0, 1), 0.35).normalize();
      this.controls.target.copy(this.tip);
      this.camera.position.copy(this.tip).addScaledVector(from, FOLLOW_DISTANCE_M);
    } else {
      this.camera.position.copy(this.wideView.position);
      this.controls.target.copy(this.wideView.target);
    }
    this.requestRender();
  }

  // Turns the camera around what it looks at, z up: by `azimuth` about the vertical and by
  // `elevation` up or down, kept short of straight above or below.
  orbit(azimuth, elevation) {
    const offset = new THREE.Vector3().subVectors(this.camera.position, this.controls.target);
    const radius = offset.length();
    const flat = Math.hypot(offset.x, offset.y);
    const heading = Math.atan2(offset.y, offset.x) + azimuth;
    const limit = 1.45;
    const pitch = Math.max(-limit, Math.min(limit, Math.atan2(offset.z, flat) + elevation));
    offset.set(
      radius * Math.cos(pitch) * Math.cos(heading),
      radius * Math.cos(pitch) * Math.sin(heading),
      radius * Math.sin(pitch),
    );
    this.camera.position.copy(this.controls.target).add(offset);
    this.camera.lookAt(this.controls.target);
    this.requestRender();
  }

  // Marks the joint the operator is turning, from 0 at the base, and the link it turns; or
  // nothing, with null. The tool keeps its own colour, which turns red on a stop.
  setChosenJoint(index) {
    if (index === this.chosenJoint) {
      return;
    }
    this.chosenJoint = index;
    this.joints.forEach((mesh, i) => {
      mesh.material = i === index ? this.chosenMaterial : this.jointMaterial;
    });
    this.links.forEach((mesh, i) => {
      if (mesh.material !== this.toolMaterial) {
        mesh.material = i === index ? this.chosenMaterial : this.linkMaterial;
      }
    });
    this.requestRender();
  }

  // `chain` is seven points: the six joint origins and the tool tip.
  setPose(chain, stopped) {
    const points = chain.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
    const tip = points[points.length - 1];
    if (this.following && this.tip !== null) {
      const moved = new THREE.Vector3().subVectors(tip, this.tip);
      this.camera.position.add(moved);
      this.controls.target.add(moved);
    }
    this.tip = tip.clone();
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
