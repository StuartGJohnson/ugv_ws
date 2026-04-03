import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { PCDLoader } from 'three/addons/loaders/PCDLoader.js';

function parseBool(v, def=false){ if(v===undefined) return def; return (v==="true"||v===true||v==="1"); }
function parseNum(v, def){ const n = Number(v); return Number.isFinite(n) ? n : def; }

export function initPCDViewer(el){
  const src = el.dataset.src;
  if(!src){ el.innerText = "PCD viewer: data-src missing"; return; }

  // Options (all optional)
  const bg          = el.dataset.bg || "#101014";
  const pointSize   = parseNum(el.dataset.pointSize, 1.5);
  const showGrid    = parseBool(el.dataset.grid, false);
  const showAxes    = parseBool(el.dataset.axes, true);
  const alpha       = parseBool(el.dataset.alpha, false); // preserve drawing buffer alpha?
  const maxPoints   = parseNum(el.dataset.maxPoints, Infinity); // simple decimation guard

  // Basic container styling
  el.style.position = el.style.position || "relative";
  if(!el.style.width)  el.style.width  = "800px";
  if(!el.style.height) el.style.height = el.style.width;
  el.style.background = bg;

  // Loading overlay
  const overlay = document.createElement("div");
  overlay.textContent = "Loading point cloud…";
  Object.assign(overlay.style, {
    position:"absolute", inset:"0", display:"flex", alignItems:"center",
    justifyContent:"center", color:"#bbb", font:"14px/1.2 system-ui, sans-serif",
    pointerEvents:"none"
  });
  el.appendChild(overlay);

  // Three.js boilerplate
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(bg);
  scene.up.set(0,0,1); // Z-up (ROS-friendly)

  const renderer = new THREE.WebGLRenderer({ antialias:true, alpha });
  renderer.setPixelRatio(window.devicePixelRatio);
  el.appendChild(renderer.domElement);

  const camera = new THREE.PerspectiveCamera(60, 1, 0.01, 1e6);
  camera.position.set(0, -3, 2);
  camera.up.set(0,0,1);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = false;
  controls.dampingFactor = 0.0;
  controls.enablePan = true;
  controls.panSpeed = 2.0;
  controls.zoomToCursor = true;
  controls.screenSpacePanning = false;

  // Helpers (added after sizing)
  let gridHelper = null;
  if(showAxes){
    const axes = new THREE.AxesHelper(0.5);
    scene.add(axes);
  }

  // Resize handling
  const resize = () => {
    const w = el.clientWidth | 0;
    const h = el.clientHeight | 0;
    if(w === 0 || h === 0) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  const ro = new ResizeObserver(resize);
  ro.observe(el);

  // Load PCD
  const loader = new PCDLoader();
  loader.load(
    src,
    (points) => {
      // Optional decimation for gigantic clouds
      if (maxPoints < Infinity && points.geometry && points.geometry.attributes.position?.count > maxPoints) {
        const pos = points.geometry.attributes.position;
        const step = Math.ceil(pos.count / maxPoints);
        const sel = [];
        for (let i=0; i<pos.count; i+=step) sel.push(i);

        // Build a reduced geometry
        const newPos = new Float32Array(sel.length * 3);
        for (let i=0; i<sel.length; i++){
          newPos[i*3+0] = pos.getX(sel[i]);
          newPos[i*3+1] = pos.getY(sel[i]);
          newPos[i*3+2] = pos.getZ(sel[i]);
        }
        const geo = new THREE.BufferGeometry();
        geo.setAttribute("position", new THREE.BufferAttribute(newPos, 3));

        // Colors (if any)
        if(points.geometry.attributes.color){
          const col = points.geometry.attributes.color;
          const newCol = new Float32Array(sel.length * 3);
          for (let i=0; i<sel.length; i++){
            newCol[i*3+0] = col.getX(sel[i]);
            newCol[i*3+1] = col.getY(sel[i]);
            newCol[i*3+2] = col.getZ(sel[i]);
          }
          geo.setAttribute("color", new THREE.BufferAttribute(newCol, 3));
        }

        points.geometry.dispose();
        points.geometry = geo;
      }

      // Material tweaks
      const mat = points.material;
      mat.size = pointSize;
      mat.sizeAttenuation = true;
      mat.vertexColors = !!points.geometry.attributes.color;
      mat.transparent = true;
      mat.opacity = 1.0;

      // Fit camera
      points.geometry.computeBoundingSphere();
      const bs = points.geometry.boundingSphere;
      const r = Math.max(1e-3, bs.radius);
      const c = bs.center;

      // Grid sized to cloud
      if (showGrid) {
        const gridSize = Math.pow(2, Math.ceil(Math.log2(r*2 || 1))) * 2; // nice round size
        gridHelper = new THREE.GridHelper(gridSize, 20, 0x777777, 0x333333);
        gridHelper.rotation.x = Math.PI/2; // put on Z-up plane (XY grid)
        scene.add(gridHelper);
      }

      scene.add(points);

      // Camera position/target for Z-up scene
      controls.target.copy(c);
      camera.position.set(c.x + r*0.3, c.y - r*2.2, c.z + r*1.0);
      camera.near = Math.max(0.01, r/1000);
      camera.far  = Math.max(1000, r*10);
      camera.updateProjectionMatrix();

      overlay.remove();
      animate();
    },
    undefined,
    (err) => {
      overlay.textContent = `Failed to load: ${src}`;
      console.error(err);
    }
  );

  function animate(){
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }

  // Double-click to refit (if bounding sphere available)
  el.addEventListener("dblclick", () => {
    const pts = scene.children.find(o => o.isPoints);
    if(!pts) return;
    pts.geometry.computeBoundingSphere();
    const bs = pts.geometry.boundingSphere;
    const r = Math.max(1e-3, bs.radius);
    const c = bs.center;
    controls.target.copy(c);
    camera.position.set(c.x + r*0.3, c.y - r*2.2, c.z + r*1.0);
  });

  // Clean-up if your page ever dynamically removes viewers:
  el._pcd_dispose = () => {
    ro.disconnect();
    renderer.dispose();
  };
}

export function createPCDViewers(){
  document.querySelectorAll("[data-pcd][data-src], .pcd-viewer[data-src]").forEach(initPCDViewer);
}

// Auto-init on DOM ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", createPCDViewers);
} else {
  createPCDViewers();
}

