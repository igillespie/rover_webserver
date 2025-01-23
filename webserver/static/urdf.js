// Scene Setup
const container = document.getElementById('urdf-container');
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();
renderer.setSize(container.clientWidth, container.clientHeight);
container.appendChild(renderer.domElement);

// Lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
directionalLight.position.set(1, 1, 1);
scene.add(ambientLight, directionalLight);

// Camera Position
camera.position.set(2, 2, 2);
camera.lookAt(0, 0, 0);

// URDF Loader
const loader = new URDFLoader();
loader.load('urdf/osr.urdf', (robot) => {
    robot.rotation.x = Math.PI / 2; // Adjust orientation if needed
    scene.add(robot);

    // Animation Loop
    function animate() {
        requestAnimationFrame(animate);
        robot.rotation.z += 0.01; // Example animation (rotate robot)
        renderer.render(scene, camera);
    }
    animate();
});

// Handle Window Resize
window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
});