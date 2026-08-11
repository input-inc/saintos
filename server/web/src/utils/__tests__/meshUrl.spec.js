// Lock-down tests for URDF mesh URL resolution (2026-08).
//
// Regression: resolveMeshUrl used to strip every reference to its
// basename, which — combined with the server's old flatten-on-upload —
// collapsed same-named meshes from different subfolders into one file.
// In the johnny5 head model, SimpleMouth/ and SimplifiedHead2/ both
// ship static_97a3da.stl (different geometry): the survivor rendered
// TWICE in the settings preview and its counterpart vanished.
import { describe, expect, it } from 'vitest'
import { resolveMeshUrl } from '../meshUrl'

const BASE = '/api/robot/meshes/'
const URDF = '/api/robot/urdf?v=1723300000'

describe('resolveMeshUrl', () => {
  it('preserves the URDF-relative subdirectory path (johnny5 regression)', () => {
    // urdf-loader resolves "Meshes/SimpleMouth/static_97a3da.stl"
    // against the URDF's URL dir before calling us.
    expect(resolveMeshUrl('/api/robot/Meshes/SimpleMouth/static_97a3da.stl', URDF, BASE))
      .toBe(BASE + 'Meshes/SimpleMouth/static_97a3da.stl')
    expect(resolveMeshUrl('/api/robot/Meshes/SimplifiedHead2/static_97a3da.stl', URDF, BASE))
      .toBe(BASE + 'Meshes/SimplifiedHead2/static_97a3da.stl')
  })

  it('two same-basename refs resolve to DIFFERENT urls', () => {
    const a = resolveMeshUrl('/api/robot/Meshes/SimpleMouth/static_97a3da.stl', URDF, BASE)
    const b = resolveMeshUrl('/api/robot/Meshes/SimplifiedHead2/static_97a3da.stl', URDF, BASE)
    expect(a).not.toBe(b)
  })

  it('strips package:// scheme and package name', () => {
    expect(resolveMeshUrl('package://my_robot/meshes/arm/link1.stl', URDF, BASE))
      .toBe(BASE + 'meshes/arm/link1.stl')
  })

  it('handles flat refs next to the urdf', () => {
    expect(resolveMeshUrl('/api/robot/wheel.stl', URDF, BASE))
      .toBe(BASE + 'wheel.stl')
  })

  it('handles ./ prefixes and query-string urdf urls', () => {
    expect(resolveMeshUrl('/api/robot/./Meshes/a.stl', URDF, BASE))
      .toBe(BASE + 'Meshes/a.stl')
  })

  it('passes through refs outside the urdf dir without prefix-stripping', () => {
    // A ref that doesn't share the URDF's URL prefix keeps its own
    // (cleaned) path — the server decides whether it exists.
    expect(resolveMeshUrl('SharedMeshes/common.stl', URDF, BASE))
      .toBe(BASE + 'SharedMeshes/common.stl')
  })

  it('encodes each segment but not the separators', () => {
    expect(resolveMeshUrl('/api/robot/Mesh Files/part #2.stl', URDF, BASE))
      .toBe(BASE + 'Mesh%20Files/part%20%232.stl')
  })

  it('tolerates empty/odd inputs without throwing', () => {
    expect(resolveMeshUrl('', URDF, BASE)).toBe(BASE)
    expect(resolveMeshUrl(null, '', BASE)).toBe(BASE)
  })
})
