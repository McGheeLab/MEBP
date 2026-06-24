"""
build_guide.py — Render the MEBP Quick Start Guide to PDF.

Uses PySide6 QPdfWriter + QPainter with a small flow-layout engine so the
document paginates cleanly with a cover page, table of contents, per-page
header/footer with page numbers, callout boxes, settings tables, and figures.

Content comes from quickstart_guide/guide_content.json (produced by the
research workflow). Screenshots are pulled from quickstart_guide/screenshots/
and quickstart_guide/assets/.

Run from the repo root:
    QT_QPA_PLATFORM=offscreen python quickstart_guide/build_guide.py
"""
import os
import sys
import json

# Use the REAL Qt platform — the offscreen plugin loads 0 system fonts on this
# build, which would render every glyph in the PDF as a tofu box. QPdfWriter
# needs no visible window, so the default "windows" platform is fine here.

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SHOTS = os.path.join(HERE, "screenshots")
ASSETS = os.path.join(HERE, "assets")
CONTENT_JSON = os.path.join(HERE, "guide_content.json")
OUT_PDF = os.path.join(HERE, "MEBP_Quick_Start_Guide.pdf")

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import (
    QPdfWriter, QPainter, QTextDocument, QFont, QColor, QPen, QBrush,
    QPageSize, QPageLayout, QImage,
)
from PySide6.QtCore import QMarginsF, QRectF, Qt, QSizeF

# ── Palette (print-friendly, professional) ─────────────────────────────
INK      = "#1b2330"   # body text
PRIMARY  = "#1f4e79"   # deep blue — headings / chrome
PRIMARY2 = "#2e6da4"   # lighter blue
ACCENT   = "#0d8a72"   # teal — step numbers / links
MUTED    = "#6b7785"   # captions / footer
RULE     = "#d6dde6"
SAFETY_BG   = "#fdecec"; SAFETY_BAR  = "#c0392b"; SAFETY_TX  = "#7d1f17"
TIP_BG      = "#e9f5ef"; TIP_BAR     = "#0d8a72"; TIP_TX     = "#0c5a4a"
INFO_BG     = "#eaf1f8"; INFO_BAR    = "#2e6da4"; INFO_TX    = "#1f3d5c"
PREREQ_BG   = "#fbf3e4"; PREREQ_BAR  = "#d08a1d"; PREREQ_TX  = "#7a5410"
ROW_ALT  = "#f3f6fa"
CARD_BG  = "#f7f9fc"

SANS = "Segoe UI"
MONO = "Consolas"

DPI = 200


def mm(v):
    return v / 25.4 * DPI


def f(size_pt, bold=False, family=SANS, italic=False):
    ft = QFont(family, 0)
    ft.setPointSizeF(size_pt)
    ft.setBold(bold)
    ft.setItalic(italic)
    return ft


def esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


class GuideDoc:
    def __init__(self, path):
        self.writer = QPdfWriter(path)
        self.writer.setPageSize(QPageSize(QPageSize.A4))
        self.writer.setResolution(DPI)
        self.MARGIN = mm(15)
        self.writer.setPageMargins(QMarginsF(15, 13, 15, 13),
                                   QPageLayout.Millimeter)
        self.painter = QPainter(self.writer)
        self.painter.setRenderHint(QPainter.Antialiasing, True)
        self.painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        self.W = self.writer.width()
        self.H = self.writer.height()
        self.header_h = mm(10)
        self.footer_h = mm(9)
        self.body_top = self.header_h
        self.body_bottom = self.H - self.footer_h
        self.x0 = 0
        self.content_w = self.W
        self.y = self.body_top
        self.page = 1
        self.doc_title = "MEBP Quick Start Guide"
        self.cur_section = ""
        self.is_cover = True
        self._toc = {}        # guide_id -> page number (filled during build)
        self._record = None   # dict to record start pages on a counting pass

    # ── page chrome ───────────────────────────────────────────────────
    def _chrome(self):
        if self.is_cover:
            return
        p = self.painter
        # Header rule + section name
        p.setPen(QPen(QColor(RULE), mm(0.3)))
        hy = self.header_h - mm(3)
        p.drawLine(0, hy, self.W, hy)
        p.setFont(f(8.5, family=SANS))
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(0, 0, self.W, hy - mm(1)),
                   Qt.AlignLeft | Qt.AlignVCenter, "  " + self.doc_title)
        p.drawText(QRectF(0, 0, self.W, hy - mm(1)),
                   Qt.AlignRight | Qt.AlignVCenter, self.cur_section + "  ")
        # Footer rule + page number
        fy = self.body_bottom + mm(3)
        p.setPen(QPen(QColor(RULE), mm(0.3)))
        p.drawLine(0, fy, self.W, fy)
        p.setFont(f(8.5))
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(0, fy, self.W, self.footer_h - mm(3)),
                   Qt.AlignLeft | Qt.AlignVCenter, "  MEBP Bioprinter · V7.5.0")
        p.drawText(QRectF(0, fy, self.W, self.footer_h - mm(3)),
                   Qt.AlignRight | Qt.AlignVCenter, "Page %d  " % self.page)

    def new_page(self, section=None):
        self.writer.newPage()
        self.page += 1
        self.is_cover = False
        if section is not None:
            self.cur_section = section
        self._chrome()
        self.y = self.body_top + mm(2)

    def ensure(self, h):
        if self.y + h > self.body_bottom:
            self.new_page()
            return True
        return False

    def space(self, h):
        self.y += h

    # ── primitives ────────────────────────────────────────────────────
    def _mk_doc(self, html, width, base_pt=10.5):
        d = QTextDocument()
        d.setDocumentMargin(0)
        ft = f(base_pt)
        d.setDefaultFont(ft)
        d.setDefaultStyleSheet(
            "body{color:%s;} p{margin:0 0 4px 0;} "
            "a{color:%s;} code{font-family:%s;color:%s;} "
            "li{margin-bottom:3px;}" % (INK, ACCENT, MONO, PRIMARY))
        d.setTextWidth(width)
        d.setHtml("<body>%s</body>" % html)
        return d

    def _draw_doc(self, d, x, y):
        self.painter.save()
        self.painter.translate(x, y)
        d.drawContents(self.painter)
        self.painter.restore()

    def para(self, html, base_pt=10.5, gap=mm(2.2), indent=0):
        d = self._mk_doc(html, self.content_w - indent, base_pt)
        h = d.size().height()
        self.ensure(h + 1)
        self._draw_doc(d, self.x0 + indent, self.y)
        self.y += h + gap

    def heading(self, text, pt=15, color=PRIMARY, gap_before=mm(3),
                gap_after=mm(2), rule=False):
        self.space(gap_before)
        # Render via QTextDocument (same path as body text) so the measured
        # height matches what's painted — painter.drawText metrics were
        # clipping ascenders / cutting the rule through the text.
        html = "<span style='color:%s'><b>%s</b></span>" % (color, esc(text))
        d = self._mk_doc(html, self.content_w, base_pt=pt)
        h = d.size().height()
        self.ensure(h + (mm(4) if rule else mm(1)))
        self._draw_doc(d, self.x0, self.y)
        self.y += h
        if rule:
            self.y += mm(1.6)
            self.painter.setPen(QPen(QColor(RULE), mm(0.4)))
            self.painter.drawLine(self.x0, self.y, self.x0 + self.content_w, self.y)
            self.y += mm(2)
        self.y += gap_after

    def section_number_title(self, number, title):
        """Big numbered section header for each guide."""
        # number chip
        chip = mm(11)
        self.ensure(chip + mm(4))
        p = self.painter
        p.save()
        p.setBrush(QBrush(QColor(PRIMARY)))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(QRectF(self.x0, self.y, chip, chip), mm(2), mm(2))
        p.setPen(QColor("#ffffff"))
        p.setFont(f(15, bold=True))
        p.drawText(QRectF(self.x0, self.y, chip, chip),
                   Qt.AlignCenter, str(number))
        p.setPen(QColor(PRIMARY))
        p.setFont(f(17, bold=True))
        p.drawText(QRectF(self.x0 + chip + mm(4), self.y, self.content_w - chip - mm(4), chip),
                   Qt.AlignLeft | Qt.AlignVCenter, title)
        p.restore()
        self.y += chip + mm(1)
        p.setPen(QPen(QColor(PRIMARY2), mm(0.6)))
        p.drawLine(self.x0, self.y, self.x0 + self.content_w, self.y)
        self.y += mm(4)

    def bullets(self, items, marker="•", base_pt=10.5):
        for it in items:
            d = self._mk_doc(esc(it) if "<" not in it else it,
                             self.content_w - mm(6), base_pt)
            h = d.size().height()
            self.ensure(h + 1)
            self.painter.save()
            self.painter.setFont(f(base_pt, bold=True))
            self.painter.setPen(QColor(ACCENT))
            self.painter.drawText(QRectF(self.x0, self.y, mm(6), h),
                                  Qt.AlignLeft | Qt.AlignTop, marker)
            self.painter.restore()
            self._draw_doc(d, self.x0 + mm(6), self.y)
            self.y += h + mm(1.4)

    def steps(self, items):
        """items: list of (title, detail)."""
        for i, (title, detail) in enumerate(items, 1):
            num_w = mm(8)
            html = "<b>%s</b>" % esc(title)
            if detail:
                html += "<br/><span style='color:%s'>%s</span>" % (INK, esc(detail))
            d = self._mk_doc(html, self.content_w - num_w, 10.5)
            h = max(d.size().height(), mm(7))
            self.ensure(h + mm(2))
            p = self.painter
            p.save()
            p.setBrush(QBrush(QColor(ACCENT)))
            p.setPen(Qt.NoPen)
            r = mm(6)
            p.drawEllipse(QRectF(self.x0, self.y + mm(0.3), r, r))
            p.setPen(QColor("#ffffff"))
            p.setFont(f(9.5, bold=True))
            p.drawText(QRectF(self.x0, self.y + mm(0.3), r, r),
                       Qt.AlignCenter, str(i))
            p.restore()
            self._draw_doc(d, self.x0 + num_w, self.y)
            self.y += h + mm(2.2)

    def settings_table(self, rows):
        """rows: list of dict(name, what_it_does, default, recommendation)."""
        name_w = self.content_w * 0.30
        desc_w = self.content_w - name_w
        pad = mm(2)
        # header
        hh = mm(7)
        self.ensure(hh + mm(6))
        p = self.painter
        p.save()
        p.setBrush(QBrush(QColor(PRIMARY)))
        p.setPen(Qt.NoPen)
        p.drawRect(QRectF(self.x0, self.y, self.content_w, hh))
        p.setPen(QColor("#ffffff"))
        p.setFont(f(9.5, bold=True))
        p.drawText(QRectF(self.x0 + pad, self.y, name_w - pad, hh),
                   Qt.AlignLeft | Qt.AlignVCenter, "Control / Setting")
        p.drawText(QRectF(self.x0 + name_w + pad, self.y, desc_w - pad, hh),
                   Qt.AlignLeft | Qt.AlignVCenter, "What it does")
        p.restore()
        self.y += hh
        alt = False
        for row in rows:
            name = row.get("name", "")
            desc = row.get("what_it_does", "")
            dft = (row.get("default") or "").strip()
            rec = (row.get("recommendation") or "").strip()
            extra = []
            if dft:
                extra.append("<b>Default:</b> %s" % esc(dft))
            if rec:
                extra.append("<b>Tip:</b> %s" % esc(rec))
            desc_html = esc(desc)
            if extra:
                desc_html += ("<br/><span style='color:%s'>%s</span>"
                              % (MUTED, " &nbsp;·&nbsp; ".join(extra)))
            dn = self._mk_doc("<b>%s</b>" % esc(name), name_w - 2 * pad, 9.7)
            dd = self._mk_doc(desc_html, desc_w - 2 * pad, 9.7)
            rh = max(dn.size().height(), dd.size().height()) + 2 * pad
            if self.y + rh > self.body_bottom:
                self.new_page()
                # repeat header
                p.save()
                p.setBrush(QBrush(QColor(PRIMARY)))
                p.setPen(Qt.NoPen)
                p.drawRect(QRectF(self.x0, self.y, self.content_w, hh))
                p.setPen(QColor("#ffffff"))
                p.setFont(f(9.5, bold=True))
                p.drawText(QRectF(self.x0 + pad, self.y, name_w - pad, hh),
                           Qt.AlignLeft | Qt.AlignVCenter, "Control / Setting")
                p.drawText(QRectF(self.x0 + name_w + pad, self.y, desc_w - pad, hh),
                           Qt.AlignLeft | Qt.AlignVCenter, "What it does")
                p.restore()
                self.y += hh
            if alt:
                p.fillRect(QRectF(self.x0, self.y, self.content_w, rh),
                           QColor(ROW_ALT))
            alt = not alt
            self._draw_doc(dn, self.x0 + pad, self.y + pad)
            self._draw_doc(dd, self.x0 + name_w + pad, self.y + pad)
            # row separators
            p.setPen(QPen(QColor(RULE), mm(0.2)))
            p.drawLine(self.x0, self.y + rh, self.x0 + self.content_w, self.y + rh)
            p.drawLine(self.x0 + name_w, self.y, self.x0 + name_w, self.y + rh)
            self.y += rh
        # outer border
        self.y += mm(3)

    def callout(self, kind, title, items):
        cfg = {
            "safety": (SAFETY_BG, SAFETY_BAR, SAFETY_TX, "⚠  "),
            "tip":    (TIP_BG, TIP_BAR, TIP_TX, "✓  "),
            "info":   (INFO_BG, INFO_BAR, INFO_TX, "ℹ  "),
            "prereq": (PREREQ_BG, PREREQ_BAR, PREREQ_TX, "▶  "),
        }[kind]
        bg, bar, tx, _icon = cfg
        pad = mm(3)
        bar_w = mm(1.6)
        inner_w = self.content_w - 2 * pad - bar_w
        body = ""
        if isinstance(items, str):
            body = "<span style='color:%s'>%s</span>" % (tx, esc(items))
        else:
            lis = "".join("<li>%s</li>" % (it if "<" in it else esc(it))
                          for it in items)
            body = "<ul style='margin:0;color:%s'>%s</ul>" % (tx, lis)
        head = "<b style='color:%s'>%s</b>" % (tx, esc(title)) if title else ""
        html = (head + ("<br/>" if head else "") + body)
        d = self._mk_doc(html, inner_w, 10)
        bh = d.size().height() + 2 * pad
        self.ensure(bh + mm(2))
        p = self.painter
        p.fillRect(QRectF(self.x0, self.y, self.content_w, bh), QColor(bg))
        p.fillRect(QRectF(self.x0, self.y, bar_w, bh), QColor(bar))
        self._draw_doc(d, self.x0 + bar_w + pad, self.y + pad)
        self.y += bh + mm(3)

    def figure(self, img_path, caption="", max_h_mm=92, border=True):
        if not os.path.exists(img_path):
            return False
        img = QImage(img_path)
        if img.isNull():
            return False
        avail_w = self.content_w
        max_h = mm(max_h_mm)
        scale = min(avail_w / img.width(), max_h / img.height())
        w = img.width() * scale
        h = img.height() * scale
        cap_doc = None
        cap_h = 0
        if caption:
            cap_doc = self._mk_doc(
                "<span style='color:%s'><i>%s</i></span>" % (MUTED, esc(caption)),
                self.content_w, 9)
            cap_h = cap_doc.size().height() + mm(1.5)
        block_h = h + cap_h + mm(2)
        # Try to keep figure+caption together
        if self.y + block_h > self.body_bottom and self.y > self.body_top + mm(5):
            self.new_page()
        x = self.x0 + (self.content_w - w) / 2
        target = QRectF(x, self.y, w, h)
        self.painter.drawImage(target, img)
        if border:
            self.painter.setPen(QPen(QColor(RULE), mm(0.4)))
            self.painter.setBrush(Qt.NoBrush)
            self.painter.drawRect(target)
        self.y += h + mm(1.5)
        if cap_doc:
            self._draw_doc(cap_doc, self.x0, self.y)
            self.y += cap_h
        self.y += mm(2)
        return True

    # ── pages ────────────────────────────────────────────────────────
    def cover(self, subtitle, date_str, hero_img=None):
        p = self.painter
        # top band
        band_h = mm(70)
        p.fillRect(QRectF(-self.MARGIN, -self.MARGIN,
                          self.W + 2 * self.MARGIN, band_h), QColor(PRIMARY))
        p.setPen(QColor("#cfe0f0"))
        p.setFont(f(11, family=SANS))
        p.drawText(QRectF(0, mm(8), self.content_w, mm(8)),
                   Qt.AlignLeft | Qt.AlignVCenter, "MICROSCOPE-ENABLED BIOPRINTING PLATFORM")
        p.setPen(QColor("#ffffff"))
        p.setFont(f(30, bold=True))
        p.drawText(QRectF(0, mm(18), self.content_w, mm(26)),
                   Qt.AlignLeft | Qt.AlignVCenter, "MEBP Quick Start Guide")
        p.setPen(QColor("#dce8f4"))
        p.setFont(f(13, family=SANS))
        p.drawText(QRectF(0, mm(44), self.content_w, mm(12)),
                   Qt.AlignLeft | Qt.AlignTop, subtitle)
        self.y = band_h + mm(6)
        self.is_cover = True
        if hero_img and os.path.exists(hero_img):
            self.figure(hero_img, "The MEBP application window: sidebar (pages) on the "
                        "left, the active page in the centre, status bar along the bottom.",
                        max_h_mm=110)
        # what's inside
        self.space(mm(2))
        self.heading("What's inside", pt=13)
        self.bullets([
            "Getting hardware set up — connect the stages, define your pumps, "
            "needle, inks and cameras.",
            "Calibration — teach the needle position, Z reference heights and the well-plate map.",
            "Quick Print — drop one object in a well and print, with every optional setting explained.",
            "Cell Targeting & Removal — trypsinize cells in place and relocate them.",
            "Spheroid Pick & Place — aspirate spheroids and deposit them at a target.",
            "Cell Staining (Labeling) — deposit a stain, incubate, then aspirate it back off.",
        ])
        self.space(mm(3))
        p.setPen(QColor(MUTED))
        p.setFont(f(9.5))
        p.drawText(QRectF(0, self.body_bottom - mm(2), self.content_w, mm(6)),
                   Qt.AlignLeft, "Generated %s · Software V7.5.0 (Version-7.5.0)" % date_str)

    def toc(self, entries):
        """entries: list of (number, title, page). Forced to one page."""
        self.new_page("Contents")
        self.heading("Table of Contents", pt=18, rule=True)
        self.space(mm(2))
        for number, title, page in entries:
            row_h = mm(9)
            p = self.painter
            p.save()
            p.setBrush(QBrush(QColor(PRIMARY if number != "" else INFO_BAR)))
            p.setPen(Qt.NoPen)
            chip = mm(7)
            if number != "":
                p.drawRoundedRect(QRectF(self.x0, self.y, chip, chip), mm(1.5), mm(1.5))
                p.setPen(QColor("#ffffff"))
                p.setFont(f(10.5, bold=True))
                p.drawText(QRectF(self.x0, self.y, chip, chip), Qt.AlignCenter, str(number))
            p.setPen(QColor(INK))
            p.setFont(f(12, bold=(number == "")))
            tx0 = self.x0 + (chip + mm(4) if number != "" else 0)
            p.drawText(QRectF(tx0, self.y, self.content_w - mm(20) - tx0, chip),
                       Qt.AlignLeft | Qt.AlignVCenter, title)
            # dotted leader + page
            if page:
                p.setPen(QColor(MUTED))
                p.setFont(f(11))
                p.drawText(QRectF(self.x0, self.y, self.content_w, chip),
                           Qt.AlignRight | Qt.AlignVCenter, str(page))
            p.restore()
            self.y += row_h
        self.space(mm(4))
        self.callout("info", "How to use this guide",
                     "Work top-to-bottom the first time: hardware setup → calibration, then "
                     "the workflow you need. Screenshots show simulated hardware; your live "
                     "view will show the real stage, plate and camera feeds.")

    def finish(self):
        self.painter.end()


# ── content assembly ───────────────────────────────────────────────────
def load_content():
    if os.path.exists(CONTENT_JSON):
        with open(CONTENT_JSON, "r", encoding="utf-8") as fh:
            return json.load(fh)
    return None


# guide id -> ordered list of (screenshot filename, caption)
FIGURES = {
    "hardware_setup": [
        ("01_hw_device.png", "Hardware Setup → Device: connect or simulate each stage, set the Z axis and motion limits."),
        ("05_hw_pump.png", "Hardware Setup → Pump: syringe/channel setup and per-pump plunger calibration."),
        ("03_hw_ink.png", "Hardware Setup → Ink: the ink library and reagent well locations."),
        ("06_hw_cameras.png", "Hardware Setup → Cameras: detect, assign and calibrate each camera."),
    ],
    "calibration": [
        ("10_cal_needle_loc.png", "Calibration → Needle Location: the side X/Y cameras used to centre the needle."),
        ("assets/needle_loc_step1.png", "Step 1 — click the needle tip's bottom-left corner in the X view."),
        ("assets/needle_loc_step4.png", "Step 4 — all corners picked; review and click Center & Save."),
        ("12_cal_plate_loc.png", "Calibration → Plate Location: teach the well-plate map."),
        ("13_cal_plate_z_autocal.png", "Calibration → Plate Z Auto-Cal: guided per-well focus to find the plate bottom."),
    ],
    "quick_print": [
        ("21_quick_print.png", "Quick Print: pick an object, click a well, open ⚙ Settings, then Print."),
    ],
    "cell_targeting": [
        ("22_cell_targeting.png", "Cell Targeting & Removal: pick removal regions and a placement target on the live view."),
    ],
    "spheroid_pickup": [
        ("23_spheroid_pickup.png", "Spheroid Pick & Place: pick source spheroids and a deposit target."),
    ],
    "cell_labeling": [
        ("24_cell_labeling.png", "Cell Staining: select the regions to stain (pick-only)."),
    ],
}

GUIDE_ORDER = ["hardware_setup", "calibration", "quick_print",
               "cell_targeting", "spheroid_pickup", "cell_labeling"]


def fig_path(rel):
    if rel.startswith("assets/"):
        return os.path.join(HERE, rel)
    return os.path.join(SHOTS, rel)


def render_guide(d, number, g):
    gid = g["id"]
    d.new_page(g.get("title", ""))
    d.section_number_title(number, g.get("title", ""))

    if g.get("overview"):
        d.para(esc(g["overview"]), base_pt=11)
    if g.get("where_to_find"):
        d.callout("info", "Where to find it", esc(g["where_to_find"]))
    if g.get("prerequisites"):
        d.callout("prereq", "Before you start", g["prerequisites"])

    # lead figure
    figs = FIGURES.get(gid, [])
    if figs:
        first = figs[0]
        d.figure(fig_path(first[0]), first[1])

    if g.get("steps"):
        d.heading("Step by step", pt=14, rule=True)
        d.steps([(s.get("title", ""), s.get("detail", "")) for s in g["steps"]])

    # mid figures (the rest)
    for rel, cap in figs[1:]:
        d.figure(fig_path(rel), cap)

    if g.get("settings"):
        d.heading("Settings & controls", pt=14, rule=True)
        d.para("<span style='color:%s'>Every adjustable control on this screen "
               "(and its ⚙ Settings popout) and what it does:</span>" % MUTED,
               base_pt=9.5, gap=mm(2))
        d.settings_table(g["settings"])

    if g.get("safety_notes"):
        d.callout("safety", "Safety & gating", g["safety_notes"])
    if g.get("tips"):
        d.callout("tip", "Tips", g["tips"])


def build(pass_record=None, toc_pages=None):
    """One render pass. Returns dict guide_id -> start page."""
    import datetime
    d = GuideDoc(OUT_PDF if toc_pages is not None else
                 os.path.join(HERE, "_pass1.pdf"))
    date_str = "2026-06-23"
    d.cover("An operator's guide to setup, calibration and the print & cell workflows.",
            date_str, hero_img=os.path.join(SHOTS, "00_main_window.png"))

    content = load_content()
    guides = {x["id"]: x for x in content["guides"]} if content else {}
    ordered = [guides[g] for g in GUIDE_ORDER if g in guides]
    # fallback if research not ready: render whatever exists
    if not ordered and content:
        ordered = content["guides"]

    # TOC (fixed 1 page)
    entries = []
    for i, g in enumerate(ordered, 1):
        pg = (toc_pages or {}).get(g["id"], "") if toc_pages else ""
        entries.append((i, g.get("title", ""), pg))
    d.toc(entries)

    starts = {}
    for i, g in enumerate(ordered, 1):
        starts[g["id"]] = d.page + 1   # next_page() will be called in render
        render_guide(d, i, g)

    d.finish()
    return starts


def main():
    app = QApplication.instance() or QApplication(sys.argv)
    content = load_content()
    if not content:
        print("No guide_content.json yet — run after the research workflow.")
        return
    # Pass 1: count start pages
    starts = build(toc_pages=None)
    # Pass 2: real build with TOC page numbers
    build(toc_pages=starts)
    try:
        os.remove(os.path.join(HERE, "_pass1.pdf"))
    except OSError:
        pass
    print("Wrote", OUT_PDF)


if __name__ == "__main__":
    main()
