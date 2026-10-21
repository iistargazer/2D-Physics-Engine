# Publishing the lab

The showcase is nine static files and a WebAssembly binary. There is no server-side
physics, no database and no API key, so "deploying" is copying nine files to anything that
serves HTTP with the right content types. This page records what is actually live, how to
reproduce it, and which routes remain untested.

## 1. What is live now

| Environment | URL | State |
|---|---|---|
| Production | <https://site-22be3251e8494dca85d1edd16f3d6ae8.freebuff.page> | deployed and verified |
| Preview | <https://site-e6426a0a0bb94c8f95438b642d3c4d88.freebuff.page> | deployed and verified |

Both were deployed from this tree's `build/web` output through [deploy/](../deploy):
`deploy/worker.js` serves the assets binding unchanged (except an immutable cache header on
`index.wasm`), `deploy/manifest.json` names it, and `deploy/make-dist.sh` stages exactly nine
files into `deploy/dist`. Verified on 2026-10-21 with curl and a real Chromium:

- `/` → `200 text/html`, 12,529 bytes (the built `index.html`);
- `index.wasm` → `200 application/wasm`, 119,171 bytes, byte-identical to the repository's
  served binary, with `cache-control: public, max-age=31536000, immutable`;
- all five frontend scripts and `showcase.css` → `200` with the expected types;
- in the browser the engine reached `RUNNING` / `ACTIVE`, simulated hundreds of fixed steps
  (`stepCount` 658 on the production URL), and the telemetry panel read the live step cost;
- the console carried no errors and no failed requests.

Two things worth knowing before you deploy somewhere else:

- **Do not rewrite `/` to `/index.html` in front of a static-assets binding.** The first
  attempt did, and the host's own `/` → `index.html`, `/index.html` → `/` normalisation
  turned it into an endless `307` loop (`net::ERR_TOO_MANY_REDIRECTS` in the browser).
  Serving the request unchanged is the whole worker.
- **`/index.html` redirects to `/`** on this host for the same reason. Share the bare origin.
- The host injects a Cloudflare analytics beacon (`static.cloudflareinsights.com`). The page
  itself still requests no third-party resources; the beacon is the host's, not the build's.

## 2. Reproduce the deployment locally

```bash
source build/emsdk/emsdk_env.sh   # once per shell; see SHOWCASE.md
bash web/build.sh                 # builds build/web/index.html + index.js + index.wasm
bash deploy/make-dist.sh          # copies exactly nine files into deploy/dist
python -m http.server 8777 --bind 127.0.0.1 --directory deploy/dist
# open http://127.0.0.1:8777/
```

`make-dist.sh` fails if any of the nine files is missing, if the directory holds anything but
nine files, or if a log slipped in. `deploy/dist/` is git-ignored: it is build output.

To redeploy on a Freebuff Sites project, point the deploy at `deploy/manifest.json`; the
manifest's `assetsDirectory` is `./dist`, so run `make-dist.sh` first.

## 3. Any other static host

The nine files (plus `Content-Type: application/wasm` for `index.wasm`) are the whole
contract. `file://` will not work: `fetch`/`WebAssembly.instantiateStreaming` need HTTP.

**GitHub Pages** — not configured on this repository, so treat this as untested instructions:

```bash
bash web/build.sh && bash deploy/make-dist.sh
git checkout --orphan gh-pages
git rm -rf --cached . >/dev/null
cp -r deploy/dist/* .
git add index.html index.js index.wasm showcase.css core.js scenes.js render.js instrument.js showcase.js
git commit -m "Publish the mechanics lab"
git push origin gh-pages
# Repository Settings -> Pages -> Source: branch gh-pages, folder / (root)
```

The orphan branch holds only the nine files; `index.wasm` is served as
`application/wasm` by Pages, which is what `instantiateStreaming` needs. Compress nothing by
hand — the host gzips.

**Netlify / Vercel / Cloudflare Pages** — deploy `deploy/dist` as a static site with no build
command. Same nine files, same requirement.

**Object storage + CDN** (S3, R2, GCS) — upload `deploy/dist`, set
`text/html`, `text/css`, `text/javascript` and `application/wasm` on the extensions, and make
`index.html` the default document.

## 4. Embedding in a game instead

Hosting is optional. The engine is header-only, and
[examples/embed/game.cpp](../examples/embed/game.cpp) is a complete fixed-timestep game loop
built with one include path:

```bash
g++ -std=c++20 -O2 -I. examples/embed/game.cpp -o build/embed_game
./build/embed_game   # steps=240 crates=12 contacts=13 y=[-0.500, 2.477] player_x=1.842
```

What a shipped game still needs — stable body handles, serialization, collision layers — is
listed in [APPLICATIONS](APPLICATIONS.md#2-what-it-cannot-do-yet).
