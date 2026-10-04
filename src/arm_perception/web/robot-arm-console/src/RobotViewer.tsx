import { useEffect, useRef } from 'react'
import { RotateCcw } from 'lucide-react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import URDFLoader from 'urdf-loader'
import type { Bootstrap, DetectedObject, JointState } from './types'

type LoadedRobot = THREE.Object3D & {
  setJointValue: (name: string, value: number) => void
}

interface Props {
  bootstrap: Bootstrap | null
  joints?: JointState
  objects: DetectedObject[]
}

export function RobotViewer({ bootstrap, joints, objects }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const robotRef = useRef<LoadedRobot | null>(null)
  const objectsRef = useRef<THREE.Group | null>(null)
  const resetViewRef = useRef<() => void>(() => undefined)

  useEffect(() => {
    const container = containerRef.current
    if (!container) return

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x0a0f15)
    scene.up.set(0, 0, 1)
    const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 20)
    camera.position.set(0.62, -0.68, 0.48)
    camera.up.set(0, 0, 1)
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.outputColorSpace = THREE.SRGBColorSpace
    container.appendChild(renderer.domElement)

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.target.set(0, 0, 0.18)
    controls.enableDamping = true
    controls.dampingFactor = 0.08
    controls.minDistance = 0.25
    controls.maxDistance = 2.2
    resetViewRef.current = () => {
      camera.position.set(0.62, -0.68, 0.48)
      controls.target.set(0, 0, 0.18)
      controls.update()
    }

    scene.add(new THREE.HemisphereLight(0xdcecff, 0x17202a, 1.8))
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.2)
    keyLight.position.set(0.7, -0.4, 1.2)
    scene.add(keyLight)
    const grid = new THREE.GridHelper(1.2, 24, 0x344150, 0x1b2632)
    grid.rotation.x = Math.PI / 2
    scene.add(grid)
    scene.add(new THREE.AxesHelper(0.12))
    const objectGroup = new THREE.Group()
    scene.add(objectGroup)
    objectsRef.current = objectGroup

    const resize = () => {
      const width = Math.max(container.clientWidth, 1)
      const height = Math.max(container.clientHeight, 1)
      camera.aspect = width / height
      camera.updateProjectionMatrix()
      renderer.setSize(width, height, false)
    }
    const observer = new ResizeObserver(resize)
    observer.observe(container)
    resize()

    let frame = 0
    const render = () => {
      controls.update()
      renderer.render(scene, camera)
      frame = requestAnimationFrame(render)
    }
    render()

    return () => {
      cancelAnimationFrame(frame)
      observer.disconnect()
      controls.dispose()
      renderer.dispose()
      renderer.domElement.remove()
      robotRef.current = null
      objectsRef.current = null
      resetViewRef.current = () => undefined
    }
  }, [])

  useEffect(() => {
    const container = containerRef.current
    const scene = container?.firstElementChild
      ? (objectsRef.current?.parent as THREE.Scene | null)
      : null
    if (!scene || !bootstrap?.robotDescription) return

    if (robotRef.current) scene.remove(robotRef.current)
    const loader = new URDFLoader()
    loader.packages = bootstrap.packageMap
    const robot = loader.parse(bootstrap.robotDescription) as LoadedRobot
    robot.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        child.castShadow = false
        child.receiveShadow = false
      }
    })
    scene.add(robot)
    robotRef.current = robot
    return () => {
      scene.remove(robot)
      if (robotRef.current === robot) robotRef.current = null
    }
  }, [bootstrap])

  useEffect(() => {
    const robot = robotRef.current
    if (!robot || !joints) return
    joints.names.forEach((name, index) => {
      const value = joints.positions[index]
      if (Number.isFinite(value)) robot.setJointValue(name, value)
    })
  }, [joints])

  useEffect(() => {
    const group = objectsRef.current
    if (!group) return
    group.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        child.geometry.dispose()
        const materials = Array.isArray(child.material) ? child.material : [child.material]
        materials.forEach((material) => material.dispose())
      }
    })
    group.clear()
    objects.forEach((object) => {
      const dimensions = object.dimensions.length >= 3
        ? object.dimensions
        : [0.025, 0.025, 0.025]
      const geometry = object.className.includes('cylinder')
        ? new THREE.CylinderGeometry(dimensions[0] / 2, dimensions[0] / 2, dimensions[2], 24)
        : new THREE.BoxGeometry(dimensions[0], dimensions[1], dimensions[2])
      if (object.className.includes('cylinder')) geometry.rotateX(Math.PI / 2)
      const color = object.className.includes('red')
        ? 0xef4444
        : object.className.includes('yellow') ? 0xfacc15 : object.className.includes('blue') ? 0x3b82f6 : 0xf97316
      const mesh = new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.82 }))
      mesh.position.set(object.position.x, object.position.y, object.position.z)
      mesh.quaternion.set(
        object.orientation.x, object.orientation.y,
        object.orientation.z, object.orientation.w)
      group.add(mesh)
    })
  }, [objects])

  return (
    <div className="robot-viewer" ref={containerRef}>
      <button
        className="viewer-control"
        type="button"
        aria-label="Dijital ikiz kamera açısını sıfırla"
        title="Görünümü sıfırla"
        onClick={() => resetViewRef.current()}
      >
        <RotateCcw size={17} />
      </button>
      {!bootstrap?.robotDescription && (
        <div className="viewer-empty" role="status">
          <span>URDF bekleniyor</span>
          <small>/robot_description henüz alınmadı</small>
        </div>
      )}
    </div>
  )
}
