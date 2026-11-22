// 3D digital twin of the arm (three.js).
//
// Role: loads the GENERATED robot description (/description/exxeta_arm.urdf + STL
// meshes) and shows the arm at the joint angles the server reports. Because the
// URDF comes from the CAD parameters, the twin changes automatically when the
// arm's dimensions change. A small built-in URDF reader is used instead of a
// library: this robot only needs revolute, prismatic and fixed joints, and the
// gripper's "mimic" joint.
//
// Look: line-drawing style to match the app: pale faces, black edges.
// The "ghost" is a second copy drawn as edges only; it shows MEASURED angles
// (pots) next to the commanded pose.
import * as THREE from 'three';
import { OrbitControls } from '../vendor/three/controls/OrbitControls.js';
import { TransformControls } from '../vendor/three/controls/TransformControls.js';
import { STLLoader } from '../vendor/three/loaders/STLLoader.js';

const URDF_URL = '/description/exxeta_arm.urdf';

function num3(s, d = [0, 0, 0]) { return s ? s.trim().split(/\s+/).map(Number) : d; }

/** Build a robot from URDF text. Returns { root, joints: {name: {object, type, axis, mimic}} }. */
async function buildRobot(urdfText, ghost) {
  const xml = new DOMParser().parseFromString(urdfText, 'application/xml');
  const stl = new STLLoader();
  const links = {}, joints = {};
  const edgeMat = new THREE.LineBasicMaterial({ color: 0x000000 });
  for (const el of xml.querySelectorAll('robot > link')) {
    const g = new THREE.Group();
    g.name = el.getAttribute('name');
    links[g.name] = g;
    const meshEl = el.querySelector('visual mesh');
    if (!meshEl) continue;
    const rgba = num3(el.querySelector('visual color')?.getAttribute('rgba'), [0.9, 0.9, 0.9, 1]);
    const dark = rgba[0] < 0.5;
    const geo = await stl.loadAsync('/description/' + meshEl.getAttribute('filename'));
    const sc = num3(meshEl.getAttribute('scale'), [1, 1, 1]);
    geo.scale(sc[0], sc[1], sc[2]);
    if (!ghost) {
      const mat = new THREE.MeshLambertMaterial({ color: dark ? 0x1c1c1c : 0xf2f2f2, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 });
      g.add(new THREE.Mesh(geo, mat));
    }
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo, 28), dark && !ghost ? new THREE.LineBasicMaterial({ color: 0x777777 }) : edgeMat);
    g.add(edges);
  }
  let root = null;
  const children = new Set();
  for (const el of xml.querySelectorAll('robot > joint')) {
    const parent = links[el.querySelector('parent').getAttribute('link')];
    const child = links[el.querySelector('child').getAttribute('link')];
    const o = el.querySelector('origin');
    const holder = new THREE.Group();                 // fixed transform parent -> joint frame
    holder.position.fromArray(num3(o?.getAttribute('xyz')));
    const rpy = num3(o?.getAttribute('rpy'));
    holder.rotation.set(rpy[0], rpy[1], rpy[2], 'ZYX');
    const mover = new THREE.Group();                  // the part that moves with the joint value
    holder.add(mover);
    mover.add(child);
    parent.add(holder);
    children.add(child.name);
    const m = el.querySelector('mimic');
    joints[el.getAttribute('name')] = {
      object: mover, type: el.getAttribute('type'),
      axis: new THREE.Vector3().fromArray(num3(el.querySelector('axis')?.getAttribute('xyz'), [0, 0, 1])),
      mimic: m ? m.getAttribute('joint') : null,
    };
  }
  for (const name in links) if (!children.has(name)) root = links[name];
  return { root, joints, links };
}

function setJoint(robot, name, value) {
  const j = robot.joints[name];
  if (!j) return;
  if (j.type === 'revolute') j.object.setRotationFromAxisAngle(j.axis, value);
  else if (j.type === 'prismatic') j.object.position.copy(j.axis).multiplyScalar(value);
  for (const other in robot.joints) if (robot.joints[other].mimic === name) setJoint(robot, other, value);
}

export class Viewer {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xffffff);
    this.camera = new THREE.PerspectiveCamera(35, 1, 0.01, 10);
    this.camera.up.set(0, 0, 1);                      // the robot's Z axis points up
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.05, 0, 0.12);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xbbbbbb, 2.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.2);
    sun.position.set(0.5, -0.8, 1.5);
    this.scene.add(sun);
    const grid = new THREE.GridHelper(0.8, 16, 0x000000, 0xbbbbbb);   // the table, 5 cm squares
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
    this.jointNames = ['base_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll'];
    this.gripperMPerRad = 0.009;
    this.setView('iso');
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this.renderer.setAnimationLoop(() => { this.controls.update(); this.renderer.render(this.scene, this.camera); });
  }

  async load() {
    const [urdf, info] = await Promise.all([
      fetch(URDF_URL).then(r => r.text()),
      fetch('/description/robot_info.json').then(r => r.json()).catch(() => ({})),
    ]);
    if (info.gripper_m_per_rad) this.gripperMPerRad = info.gripper_m_per_rad;
    const robot = await buildRobot(urdf, false);
    this.ghost = await buildRobot(urdf, true);
    this.robot = robot;
    this.ghost.root.visible = false;
    this.scene.add(this.robot.root, this.ghost.root);
    this.makeTarget();
  }

  /** q = 6 joint values (rad; the 6th is the gripper SERVO angle). */
  setPose(q, ghostQ) {
    if (!this.robot || !this.ghost) return;
    this.jointNames.forEach((n, i) => setJoint(this.robot, n, q[i]));
    setJoint(this.robot, 'gripper', q[5] * this.gripperMPerRad);
    const show = !!ghostQ;
    this.ghost.root.visible = show;
    if (show) {
      this.jointNames.forEach((n, i) => setJoint(this.ghost, n, ghostQ[i]));
      setJoint(this.ghost, 'gripper', ghostQ[5] * this.gripperMPerRad);
    }
  }

  setView(name) {
    const t = this.controls.target, d = 0.85;
    const dir = { iso: [0.75, -0.9, 0.55], front: [1, 0, 0.12], side: [0, -1, 0.12], top: [0.001, 0, 1] }[name] || [1, -1, 1];
    const v = new THREE.Vector3(...dir).normalize().multiplyScalar(d);
    this.camera.position.set(t.x + v.x, t.y + v.y, t.z + v.z);
    this.controls.update();
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  // ----- IK target: a small cross you can drag in 3D (desktop) -----
  makeTarget() {
    const g = new THREE.Group();
    const mat = new THREE.LineBasicMaterial({ color: 0x000000 });
    const s = 0.018;
    const pts = [[-s, 0, 0], [s, 0, 0], [0, -s, 0], [0, s, 0], [0, 0, -s], [0, 0, s]].map(p => new THREE.Vector3(...p));
    g.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), mat));
    g.add(new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.012, 0.012), new THREE.MeshBasicMaterial({ color: 0x000000 })));
    g.visible = false;
    this.target = g;
    this.scene.add(g);
    this.gizmo = new TransformControls(this.camera, this.renderer.domElement);
    this.gizmo.setSize(0.9);
    this.gizmo.attach(g);
    this.gizmo.enabled = false;
    this.gizmoHelper = this.gizmo.getHelper();     // three r169+: the visible part of the control
    this.gizmoHelper.visible = false;
    this.gizmo.addEventListener('dragging-changed', e => { this.controls.enabled = !e.value; });
    this.gizmo.addEventListener('objectChange', () => this.onTargetMove && this.onTargetMove(g.position));
    this.scene.add(this.gizmoHelper);
  }

  showTarget(on, pos) {
    if (!this.target) return;
    if (pos) this.target.position.set(pos.x, pos.y, pos.z);
    this.target.visible = this.gizmoHelper.visible = this.gizmo.enabled = on;
  }
}
