// Mesh URL resolution for the URDF viewer.
//
// URDF references meshes with either `package://pkg/path/foo.stl` or
// plain relative paths. The server preserves each mesh's URDF-relative
// path on upload (it used to flatten to basename, which collapsed
// same-named files from different subfolders into one — johnny5's
// SimpleMouth/ and SimplifiedHead2/ both ship a static_97a3da.stl and
// the survivor rendered twice while its counterpart vanished). So:
// recover the URDF-relative path and pass it through segment-encoded.
//
// urdf-loader hands us the reference already resolved against the
// URDF's URL directory, so that prefix is stripped back off first.
// The server keeps a basename fallback for models installed before
// path preservation, so legacy flat installs still resolve.
export function resolveMeshUrl (path, urdfUrl, meshesBase) {
  let rel = String(path ?? '')
  const pkg = rel.match(/^package:\/\/[^/]+\/(.*)$/)
  if (pkg) {
    rel = pkg[1]
  } else {
    const workingPath = String(urdfUrl || '').split('?')[0].replace(/[^/]*$/, '')
    if (workingPath && rel.startsWith(workingPath)) rel = rel.slice(workingPath.length)
  }
  rel = rel.replace(/^(\.\/)+/, '').replace(/^\/+/, '')
  return meshesBase + rel.split('/').map(encodeURIComponent).join('/')
}
