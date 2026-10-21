// ============================================================================
// worker.js - the only server this showcase needs.
//
// The physics runs in WebAssembly in the browser, so hosting is purely static:
// this Worker answers every request from the nine built assets and never sees a
// simulation. Deploy with `bash deploy/make-dist.sh` first (it copies the nine
// files out of build/web into deploy/dist, which is what assetsDirectory points
// at).
//
// Do not rewrite "/" to "/index.html" here: the assets binding already resolves
// "/" to index.html and redirects "/index.html" back to "/", so a rewrite in
// front of it is an endless 307 loop (measured on the first deployment).
// ============================================================================
export default {
  async fetch(request, env) {
    const { pathname } = new URL(request.url);
    const response = await env.ASSETS.fetch(request);
    // The wasm binary is content-addressed by the build, so it can outlive the
    // page; everything else keeps the binding's own cache headers.
    if (pathname === '/index.wasm' && response.ok) {
      const headers = new Headers(response.headers);
      headers.set('cache-control', 'public, max-age=31536000, immutable');
      return new Response(response.body, { status: response.status, headers });
    }
    return response;
  }
};
