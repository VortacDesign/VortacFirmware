/* Vortac Firmware documentation — shared behavior.
 *
 * Adding a new chapter: create the HTML page, then add ONE entry to PAGES
 * below — the sidebar on every page updates automatically.
 */

const PAGES = [
  { file: "index.html",           title: "Overview" },
  { file: "setup.html",           title: "Installation & Setup" },
  { file: "configuration.html",   title: "Configuration" },
  { file: "calibration.html",     title: "Calibration" },
  { file: "print-calibration.html", title: "Print Calibration" },
  { file: "first-print.html",     title: "First Print" },
  { file: "klipperscreen.html",   title: "KlipperScreen" },
  { file: "modules.html",         title: "Module Reference" },
  { file: "gcode.html",           title: "G-code Reference" },
  { file: "troubleshooting.html", title: "Troubleshooting" },
  { file: "development.html",     title: "Development" },
];

(function () {
  "use strict";

  function store(key, value) {
    try {
      if (value === null) localStorage.removeItem(key);
      else localStorage.setItem(key, value);
    } catch (e) { /* storage unavailable (some file:// contexts) */ }
  }
  function load(key) {
    try { return localStorage.getItem(key); } catch (e) { return null; }
  }

  /* ---------- theme ---------- */

  const savedTheme = load("vortac-docs-theme");
  if (savedTheme === "dark" ||
      (savedTheme === null && window.matchMedia &&
       window.matchMedia("(prefers-color-scheme: dark)").matches)) {
    document.documentElement.setAttribute("data-theme", "dark");
  }

  function toggleTheme() {
    const dark = document.documentElement.getAttribute("data-theme") === "dark";
    if (dark) document.documentElement.removeAttribute("data-theme");
    else document.documentElement.setAttribute("data-theme", "dark");
    store("vortac-docs-theme", dark ? "light" : "dark");
  }

  /* ---------- sidebar ---------- */

  function currentFile() {
    const parts = window.location.pathname.split("/");
    return parts[parts.length - 1] || "index.html";
  }

  function buildSidebar() {
    const nav = document.querySelector("nav.sidebar");
    if (!nav) return;
    const current = currentFile();

    let html =
      '<a class="brand" href="index.html"><span>Vortac</span> Firmware Docs</a>' +
      '<p class="tagline">Klipper plugins &amp; configs for the Vortac Toolchanger</p><ul>';

    for (const p of PAGES) {
      const active = p.file === current;
      html += '<li><a href="' + p.file + '"' + (active ? ' class="active"' : "") + ">" +
              p.title + "</a>";
      if (active) {
        // on-page table of contents from h2 headings
        const heads = document.querySelectorAll("main h2[id]");
        if (heads.length > 1) {
          html += '<ul class="toc">';
          heads.forEach(function (h) {
            html += '<li><a href="#' + h.id + '">' +
                    h.textContent.replace(/#$/, "").trim() + "</a></li>";
          });
          html += "</ul>";
        }
      }
      html += "</li>";
    }
    html += "</ul>";
    html += '<button class="theme-toggle" type="button">◐ Toggle light / dark</button>';
    nav.innerHTML = html;

    nav.querySelector(".theme-toggle").addEventListener("click", toggleTheme);

    // highlight the TOC entry for the section in view
    const tocLinks = nav.querySelectorAll(".toc a");
    if (tocLinks.length && "IntersectionObserver" in window) {
      const byId = {};
      tocLinks.forEach(function (a) { byId[a.getAttribute("href").slice(1)] = a; });
      const io = new IntersectionObserver(function (entries) {
        entries.forEach(function (en) {
          if (en.isIntersecting) {
            tocLinks.forEach(function (a) { a.classList.remove("active"); });
            const a = byId[en.target.id];
            if (a) a.classList.add("active");
          }
        });
      }, { rootMargin: "0px 0px -70% 0px" });
      document.querySelectorAll("main h2[id]").forEach(function (h) { io.observe(h); });
    }
  }

  /* ---------- heading anchors ---------- */

  function addAnchors() {
    document.querySelectorAll("main h2[id], main h3[id]").forEach(function (h) {
      const a = document.createElement("a");
      a.className = "anchor";
      a.href = "#" + h.id;
      a.textContent = "#";
      h.appendChild(a);
    });
  }

  /* ---------- persistent checklists ---------- */

  function initChecklists() {
    const page = currentFile();
    document.querySelectorAll('input[type="checkbox"][data-ck]').forEach(function (box) {
      const key = "vortac-docs-ck:" + page + ":" + box.dataset.ck;
      if (load(key) === "1") box.checked = true;
      box.addEventListener("change", function () {
        store(key, box.checked ? "1" : null);
      });
    });
    document.querySelectorAll(".checklist .reset").forEach(function (btn) {
      btn.addEventListener("click", function () {
        btn.closest(".checklist")
           .querySelectorAll('input[type="checkbox"][data-ck]')
           .forEach(function (box) {
             box.checked = false;
             store("vortac-docs-ck:" + page + ":" + box.dataset.ck, null);
           });
      });
    });
  }

  /* ---------- filterable tables (gcode reference) ---------- */

  function initFilters() {
    document.querySelectorAll(".filterbox[data-target]").forEach(function (input) {
      const table = document.getElementById(input.dataset.target);
      if (!table) return;
      input.addEventListener("input", function () {
        const q = input.value.trim().toLowerCase();
        table.querySelectorAll("tbody tr").forEach(function (tr) {
          tr.style.display = !q || tr.textContent.toLowerCase().includes(q) ? "" : "none";
        });
      });
    });
  }

  /* ---------- prev/next footer navigation ---------- */

  function buildPageNav() {
    const holder = document.querySelector(".page-nav[data-auto]");
    if (!holder) return;
    const idx = PAGES.findIndex(function (p) { return p.file === currentFile(); });
    if (idx < 0) return;
    let html = "";
    if (idx > 0) {
      const p = PAGES[idx - 1];
      html += '<a href="' + p.file + '"><span class="dir">← Previous</span>' + p.title + "</a>";
    }
    if (idx < PAGES.length - 1) {
      const p = PAGES[idx + 1];
      html += '<a class="next" href="' + p.file + '"><span class="dir">Next →</span>' + p.title + "</a>";
    }
    holder.innerHTML = html;
  }

  document.addEventListener("DOMContentLoaded", function () {
    buildSidebar();
    addAnchors();
    initChecklists();
    initFilters();
    buildPageNav();
  });
})();
