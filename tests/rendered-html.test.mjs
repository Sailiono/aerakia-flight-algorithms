import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const projectRoot = new URL("../", import.meta.url);

test("the workstation source exposes both analysis views and provenance labels", async () => {
  const [page, layout, css, packageJson] = await Promise.all([
    readFile(new URL("../app/page.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/layout.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/globals.css", import.meta.url), "utf8"),
    readFile(new URL("../package.json", import.meta.url), "utf8"),
  ]);
  assert.match(page, /validation-data\.json/);
  assert.match(page, /覆盖矩阵/);
  assert.match(page, /证据等级/);
  assert.match(page, /独立真值/);
  assert.match(page, /PX4、机载姿态/);
  assert.match(css, /\.summary-grid/);
  assert.match(css, /\.coverage-grid/);
  assert.match(layout, /Aerakia Validation Workstation/);
  assert.match(packageJson, /"data:build"/);
  await assert.rejects(() => readFile(new URL("public/_sites-preview/SkeletonPreview.tsx", projectRoot)));
});
