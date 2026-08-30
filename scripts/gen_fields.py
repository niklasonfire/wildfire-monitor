#!/usr/bin/env python3
"""Generate every decoder from the Field Table.

ADR-0002: the meaning of every byte the Controller and the BMS send is declared
once, in `field-table.json`, and the firmware decoder, the offline decoder and
the field documentation are all produced from it. None of the three is edited
by hand. This is the thing that produces them.

    scripts/gen_fields.py --c-dir build/gen          # wf_fields.h, wf_fields.c
    scripts/gen_fields.py --py-dir build-host/gen    # wf_fields.py
    scripts/gen_fields.py --doc docs/field-table.md

The C artefacts go into a build directory and are gitignored, so nothing can be
edited into them and survive. The document is committed, because a document
nobody can read on the way past is not documentation, and `make test` asserts
that what is committed is what this generator produces.

Python standard library only, on purpose: this runs both on a development
machine and inside the ESP-IDF container, and neither is guaranteed anything
beyond stdlib. That is also why the table is JSON and not YAML.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_TABLE = os.path.join(ROOT, "field-table.json")

BANNER_C = """\
/*
 * GENERATED FILE - DO NOT EDIT.
 *
 * Produced by {generator} from {table}.
 * Edit the Field Table and rebuild; anything written here is lost.
 *
 * ADR-0002: one description of the bytes, three artefacts generated from it -
 * this decoder, the Python one the offline tools use, and {doc}.
 */
"""

BANNER_PY = '''\
"""GENERATED FILE - DO NOT EDIT.

Produced by {generator} from {table}.
Edit the Field Table and rebuild; anything written here is lost.

ADR-0002: one description of the bytes, three artefacts generated from it -
this decoder, the C one the Monitor runs, and {doc}.
"""
'''


# --------------------------------------------------------------- the table

def load(path):
    """Reads the Field Table and fills in every default, so that everything
    below can assume each field carries every key."""
    with open(path, "r", encoding="utf-8") as f:
        table = json.load(f)

    conf_ids = [c["id"] for c in table["confidence"]]

    for dev in table["devices"]:
        groups = dev.get("groups", [])
        for g in groups:
            g["key"] = g.get("frame_type", g.get("name"))
        default_group = groups[0]["key"] if len(groups) == 1 else None

        for f in dev["fields"]:
            f["device"] = dev["id"]
            f.setdefault("group", default_group)
            if f["group"] is None:
                raise SystemExit(f"{f['name']}: no group, and the device has "
                                 f"more than one to guess from")
            if f["group"] not in [g["key"] for g in groups]:
                raise SystemExit(f"{f['name']}: unknown group {f['group']}")
            if f["confidence"] not in conf_ids:
                raise SystemExit(f"{f['name']}: unknown confidence "
                                 f"{f['confidence']}")
            # Controller fields are keyed by a payload byte offset, BMS fields
            # by a register index. One key, two spaces.
            if "offset" in f:
                f["key"] = f["offset"]
            elif "register" in f:
                f["key"] = f["register"]
            else:
                raise SystemExit(f"{f['name']}: neither offset nor register")
            f.setdefault("count", 1)
            f.setdefault("width", 1)
            f.setdefault("mask", None)
            f.setdefault("shift", 0)
            f.setdefault("bias", 0)
            f.setdefault("scale", 1)
            f.setdefault("unit", "")
            f.setdefault("note", "")
            f["decimals"] = decimals_for(f["scale"])
            f["divisor"] = divisor_for(f["scale"])
            if f["ctype"] == "bool" and (f["mask"] is None or f["scale"] != 1):
                raise SystemExit(f"{f['name']}: a bool field needs a mask and "
                                 f"no scale")
    return table


def decimals_for(scale):
    """How many decimal places a value at this scale actually has. Both
    generated decoders format to exactly this many when they dump a field, and
    the cross-language test compares the text, so this is what stops C's float
    and Python's double disagreeing in a digit neither of them means."""
    d, s = 0, float(scale)
    while abs(s - round(s)) > 1e-12 and d < 6:
        s *= 10.0
        d += 1
    return d


def divisor_for(scale):
    """A scale of 0.1 is emitted as a division by 10 rather than a
    multiplication by 0.1, because 0.1 is not a float and dividing keeps the
    generated C bit-identical to the hand-written code it replaces."""
    if scale == 1:
        return None
    inv = 1.0 / float(scale)
    if abs(inv - round(inv)) < 1e-12:
        return int(round(inv))
    return None


def dev_by_id(table, dev_id):
    for d in table["devices"]:
        if d["id"] == dev_id:
            return d
    raise SystemExit(f"no device {dev_id} in the table")


def fields_of(dev, group_key):
    return [f for f in dev["fields"] if f["group"] == group_key]


def hand_written_after(dev, group_key):
    return [b for b in dev.get("hand_written", [])
            if b.get("after_group") == group_key]


def reg_needed(dev):
    """One past the highest register any field reads: a response shorter than
    this carries none of them and is rejected whole."""
    return max(f["key"] + f["count"] for f in dev["fields"])


def prefix(dev):
    return "WF_CTRL" if dev["id"] == "controller" else "WF_BMS"


# ------------------------------------------------------- value expressions

def c_read(f, src):
    """The raw register or payload bytes, before mask, sign, bias or scale."""
    if f["device"] == "bms":
        return f"{src}[{f['key']}]"
    if f["width"] == 1:
        return f"{src}[{f['key']}]"
    rd = "wf_rd_u16be" if f["endian"] == "be" else "wf_rd_u16le"
    return f"{rd}(&{src}[{f['key']}])"


def c_extract(f, src, index=None):
    """Raw bytes, masked, shifted and sign-extended - everything up to the
    point where bias and scale turn it into a number with a unit."""
    if index is not None:
        expr = f"{src}[{f['key']} + {index}]"
    else:
        expr = c_read(f, src)
    if f["mask"] is not None:
        expr = f"({expr} & 0x{f['mask']:02x}u)"
        if f["shift"]:
            expr = f"({expr} >> {f['shift']})"
    if f["signed"]:
        expr = f"({'int8_t' if f['width'] == 1 and f['device'] != 'bms' else 'int16_t'})({expr})"
    return expr


def c_value(f, src, index=None):
    """The full expression, ready to assign to the struct member."""
    raw = c_extract(f, src, index)
    if f["ctype"] == "bool":
        return f"({raw} != 0)"
    if f["ctype"] == "float":
        expr = f"(float){raw}" if not raw.startswith("(") else f"(float)({raw})"
        if f["bias"]:
            sign = "-" if f["bias"] < 0 else "+"
            expr = f"({expr} {sign} {abs(f['bias'])}.0f)"
        if f["divisor"] is not None:
            expr = f"{expr} / {f['divisor']}.0f"
        elif f["scale"] != 1:
            expr = f"{expr} * {f['scale']}f"
        return expr
    # An integer member: bias in 32 bits so that reg - 40 cannot wrap, then
    # narrow to what the member actually is.
    if f["bias"]:
        sign = "-" if f["bias"] < 0 else "+"
        expr = f"(int32_t){raw} {sign} {abs(f['bias'])}"
        return f"({f['ctype']})({expr})"
    return f"({f['ctype']})({raw})"


def py_extract(f, src, index=None):
    if index is not None:
        expr = f"{src}[{f['key']} + {index}]"
    else:
        expr = f"{src}[{f['key']}]"
        if f["device"] != "bms" and f["width"] == 2:
            rd = "_u16be" if f["endian"] == "be" else "_u16le"
            expr = f"{rd}({src}, {f['key']})"
    if f["mask"] is not None:
        expr = f"({expr} & 0x{f['mask']:02x})"
        if f["shift"]:
            expr = f"({expr} >> {f['shift']})"
    if f["signed"]:
        expr = f"_s8({expr})" if (f["width"] == 1 and f["device"] != "bms") \
            else f"_s16({expr})"
    return expr


def py_value(f, src, index=None):
    raw = py_extract(f, src, index)
    if f["ctype"] == "bool":
        return f"({raw}) != 0"
    if f["bias"]:
        sign = "-" if f["bias"] < 0 else "+"
        raw = f"({raw} {sign} {abs(f['bias'])})"
    if f["divisor"] is not None:
        return f"{raw} / {f['divisor']}"
    if f["scale"] != 1:
        return f"{raw} * {f['scale']}"
    return raw


# ------------------------------------------------------------------- the C

def c_struct(dev):
    out = [f"typedef struct {{"]
    for block in hand_written_after(dev, None):
        for m in block["members"]:
            out.append(f"    {c_member(m)}")
        out.append("")
    for g in dev["groups"]:
        if g.get("valid_flag"):
            out.append(f"    bool     {g['valid_flag']};"
                       f"      /* frame type {g['key']} seen at least once */")
        for f in fields_of(dev, g["key"]):
            arr = f"[{f['count_define']}]" if f["count"] > 1 else ""
            out.append(f"    {f['ctype']:<9}{f['name']}{arr};")
        for block in hand_written_after(dev, g["key"]):
            for m in block["members"]:
                out.append(f"    {c_member(m)}   /* hand-written */")
        out.append("")
    while out and out[-1] == "":
        out.pop()
    out.append(f"}} {dev['struct']};")
    return "\n".join(out)


def c_member(m):
    arr = f"[{m['array']}]" if m.get("array") else ""
    return f"{m['ctype']:<9}{m['name']}{arr};"


def c_field_row(f):
    mask = "0" if f["mask"] is None else f"0x{f['mask']:02x}"
    ftype = f["group"] if f["device"] == "controller" else "0"
    conf = f"WF_CONF_{f['confidence'].upper()}"
    return ("    { %-20s %-8s WF_DEV_%s, %s, %d, %d, %d, %s, %s, %s, %d, %d, "
            "%s, %d, %s }," % (
                '"%s",' % f["name"], '"%s",' % f["unit"],
                "CONTROLLER" if f["device"] == "controller" else "BMS",
                ftype, f["key"], f["width"], f["count"],
                "true" if f["endian"] == "be" else "false",
                "true" if f["signed"] else "false",
                mask, f["shift"], f["bias"], repr(float(f["scale"])),
                f["decimals"], conf))


def emit_c(table, out_dir):
    ctrl = dev_by_id(table, "controller")
    bms = dev_by_id(table, "bms")
    banner = BANNER_C.format(generator=table["generator"],
                             table=table["source_file"], doc=table["doc_file"])

    h = [banner, "#ifndef WF_FIELDS_H", "#define WF_FIELDS_H", "",
         "#include <stdbool.h>", "#include <stddef.h>", "#include <stdint.h>",
         ""]

    h.append("/* Confidence, as CONTEXT.md defines it: how well an entry is")
    h.append(" * established, visible everywhere the field appears. */")
    h.append("typedef enum {")
    for i, c in enumerate(table["confidence"]):
        h.append(f"    {c['c']} = {i},    /* {c['summary']} */")
    h.append("} wf_confidence_t;")
    h.append("")
    h.append("/* \"taken on trust from elsewhere\" and the rest, for printing. */")
    h.append("const char *wf_confidence_summary(wf_confidence_t c);")
    h.append("")
    h.append("typedef enum {")
    h.append("    WF_DEV_CONTROLLER = 0,")
    h.append("    WF_DEV_BMS = 1,")
    h.append("} wf_device_t;")
    h.append("")
    h.append("/* One Field Table entry, as data. The decoders below are generated")
    h.append(" * straight-line code rather than interpreters of this, but the")
    h.append(" * table travels with them so that anything wanting to report on a")
    h.append(" * field - its unit, its Confidence - reads it from the one source")
    h.append(" * rather than repeating it. */")
    h.append("typedef struct {")
    h.append("    const char     *name;")
    h.append("    const char     *unit;")
    h.append("    wf_device_t     device;")
    h.append("    uint8_t         frame_type;  /* Controller only; 0 for the BMS */")
    h.append("    uint16_t        key;         /* payload byte offset, or register index */")
    h.append("    uint8_t         width;       /* bytes read, or registers read */")
    h.append("    uint16_t        count;       /* 1, or the length of an array field */")
    h.append("    bool            big_endian;")
    h.append("    bool            is_signed;")
    h.append("    uint32_t        mask;        /* 0 when the whole value is taken */")
    h.append("    uint8_t         shift;")
    h.append("    int32_t         bias;        /* added to the raw value before scaling */")
    h.append("    double          scale;")
    h.append("    uint8_t         decimals;    /* how many the scale actually has */")
    h.append("    wf_confidence_t confidence;")
    h.append("} wf_field_t;")
    h.append("")
    h.append("/* Where a dumped field goes. The two dump functions below walk the")
    h.append(" * table in order and hand every field to this, which is what lets")
    h.append(" * the replay harness print exactly what the Python decoder prints. */")
    h.append("typedef void (*wf_field_sink_t)(void *ctx, const char *name,")
    h.append("                                double value, int decimals);")
    h.append("")

    for dev in (ctrl, bms):
        p = prefix(dev)
        h.append(f"/* ------------------------------------------- {dev['name']} */")
        h.append("")
        for g in dev["groups"]:
            if g.get("c_define"):
                h.append(f"#define {g['c_define']:<24} {g['key']}")
        for c in dev.get("constants", []):
            h.append(f"#define {c['name']:<24} {c['value']}")
        for f in dev["fields"]:
            if f["count"] > 1:
                h.append(f"#define {f['count_define']:<24} {f['count']}")
        if dev["id"] == "bms":
            h.append(f"#define {'WF_BMS_REG_NEEDED':<24} {reg_needed(dev)}"
                     "   /* one past the highest register any field reads */")
        h.append(f"#define {p + '_FIELD_COUNT':<24} {len(dev['fields'])}")
        h.append("")
        h.append(f"extern const wf_field_t {p.lower()}_field_table"
                 f"[{p}_FIELD_COUNT];")
        h.append("")
        h.append(c_struct(dev))
        h.append("")

    h.append("/* Folds one validated frame into live. Frame types the table does")
    h.append(" * not cover are ignored. Short enough to run inside a spinlock,")
    h.append(" * which is where the Monitor calls it from. */")
    h.append(f"void {ctrl['apply_fn']}({ctrl['struct']} *live, uint8_t type,")
    h.append("                          const uint8_t *payload);")
    h.append("")
    h.append("/* Fills out from the registers of one validated 0xd2 response. A")
    h.append(" * response shorter than WF_BMS_REG_NEEDED is left alone. */")
    h.append(f"void {bms['apply_fn']}({bms['struct']} *out, const uint16_t *reg,")
    h.append("                         size_t n_reg);")
    h.append("")
    h.append("/* Every field the given frame type carries, in table order. */")
    h.append(f"void {ctrl['dump_fn']}(const {ctrl['struct']} *live, uint8_t type,")
    h.append("                         wf_field_sink_t sink, void *ctx);")
    h.append("")
    h.append("/* Every BMS field, in table order, arrays expanded. */")
    h.append(f"void {bms['dump_fn']}(const {bms['struct']} *b,")
    h.append("                        wf_field_sink_t sink, void *ctx);")
    h.append("")
    h.append("#endif /* WF_FIELDS_H */")

    c = [banner, '#include "wf_fields.h"', "",
         "/* ------------------------ helpers */", ""]
    # Only the readers some field actually uses: an unused static function is
    # a warning, and this build treats warnings as errors.
    wide = [f for d in (ctrl, bms) for f in d["fields"]
            if f["device"] == "controller" and f["width"] == 2]
    if any(f["endian"] == "le" for f in wide):
        c.append("static uint16_t wf_rd_u16le(const uint8_t *p)")
        c.append("{")
        c.append("    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));")
        c.append("}")
        c.append("")
    if any(f["endian"] == "be" for f in wide):
        c.append("static uint16_t wf_rd_u16be(const uint8_t *p)")
        c.append("{")
        c.append("    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);")
        c.append("}")
        c.append("")
    c.append("static const char *const wf_confidence_text[] = {")
    for conf in table["confidence"]:
        c.append(f'    "{conf["summary"]}",')
    c.append("};")
    c.append("")
    c.append("const char *wf_confidence_summary(wf_confidence_t c)")
    c.append("{")
    c.append("    return ((int)c >= 0 && (size_t)c < sizeof(wf_confidence_text) /")
    c.append("            sizeof(wf_confidence_text[0])) ? wf_confidence_text[c]")
    c.append('                                           : "unknown";')
    c.append("}")
    c.append("")

    for dev in (ctrl, bms):
        p = prefix(dev)
        c.append(f"const wf_field_t {p.lower()}_field_table"
                 f"[{p}_FIELD_COUNT] = {{")
        for f in dev["fields"]:
            c.append(c_field_row(f))
        c.append("};")
        c.append("")

    # Controller: one switch case per frame type.
    c.append(f"void {ctrl['apply_fn']}({ctrl['struct']} *live, uint8_t type,")
    c.append("                          const uint8_t *p)")
    c.append("{")
    c.append("    if (live == NULL || p == NULL) {")
    c.append("        return;")
    c.append("    }")
    c.append("    switch (type) {")
    for g in ctrl["groups"]:
        c.append(f"    case {g['c_define']}:")
        for f in fields_of(ctrl, g["key"]):
            c.append(f"        live->{f['name']} = {c_value(f, 'p')};")
        if g.get("valid_flag"):
            c.append(f"        live->{g['valid_flag']} = true;")
        c.append("        break;")
        c.append("")
    c.append("    default:")
    c.append("        break;")
    c.append("    }")
    c.append("}")
    c.append("")

    # BMS: straight-line, one response decodes whole or not at all.
    c.append(f"void {bms['apply_fn']}({bms['struct']} *out, const uint16_t *reg,")
    c.append("                         size_t n_reg)")
    c.append("{")
    c.append("    if (out == NULL || reg == NULL || n_reg < (size_t)WF_BMS_REG_NEEDED) {")
    c.append("        return;")
    c.append("    }")
    for f in bms["fields"]:
        if f["count"] > 1:
            c.append(f"    for (unsigned i = 0; i < (unsigned){f['count_define']}; i++) {{")
            c.append(f"        out->{f['name']}[i] = {c_value(f, 'reg', 'i')};")
            c.append("    }")
        else:
            c.append(f"    out->{f['name']} = {c_value(f, 'reg')};")
    c.append("}")
    c.append("")

    for dev in (ctrl, bms):
        for f in dev["fields"]:
            if f["count"] > 1:
                c.append(f"static const char *const {f['name']}_names"
                         f"[{f['count_define']}] = {{")
                names = [f'"{f["name"]}[{i}]"' for i in range(f["count"])]
                for chunk in wrap_list(names, 72, "    "):
                    c.append(chunk)
                c.append("};")
                c.append("")

    c.append(f"void {ctrl['dump_fn']}(const {ctrl['struct']} *live, uint8_t type,")
    c.append("                         wf_field_sink_t sink, void *ctx)")
    c.append("{")
    c.append("    switch (type) {")
    for g in ctrl["groups"]:
        c.append(f"    case {g['c_define']}:")
        for f in fields_of(ctrl, g["key"]):
            c.append(f'        sink(ctx, "{f["name"]}", (double)live->{f["name"]},'
                     f" {f['decimals']});")
        c.append("        break;")
        c.append("")
    c.append("    default:")
    c.append("        break;")
    c.append("    }")
    c.append("}")
    c.append("")
    c.append(f"void {bms['dump_fn']}(const {bms['struct']} *b,")
    c.append("                        wf_field_sink_t sink, void *ctx)")
    c.append("{")
    for f in bms["fields"]:
        if f["count"] > 1:
            c.append(f"    for (unsigned i = 0; i < (unsigned){f['count_define']}; i++) {{")
            c.append(f"        sink(ctx, {f['name']}_names[i], (double)b->{f['name']}[i],"
                     f" {f['decimals']});")
            c.append("    }")
        else:
            c.append(f'    sink(ctx, "{f["name"]}", (double)b->{f["name"]},'
                     f" {f['decimals']});")
    c.append("}")

    write(os.path.join(out_dir, "wf_fields.h"), "\n".join(h) + "\n")
    write(os.path.join(out_dir, "wf_fields.c"), "\n".join(c) + "\n")


def wrap_list(items, width, indent):
    """Comma-separated initialisers, wrapped. The commas matter: two adjacent
    string literals in C concatenate silently, so a dropped one turns a
    28-entry name array into one long string and an out-of-bounds read."""
    out, line = [], indent
    for i, part in enumerate(items):
        piece = part + ("," if i + 1 < len(items) else "")
        if line != indent and len(line) + 1 + len(piece) > width:
            out.append(line)
            line = indent + piece
        else:
            line += (" " if line != indent else "") + piece
    if line.strip():
        out.append(line)
    return out


# -------------------------------------------------------------- the Python

def emit_py(table, out_dir):
    ctrl = dev_by_id(table, "controller")
    bms = dev_by_id(table, "bms")
    o = [BANNER_PY.format(generator=table["generator"],
                          table=table["source_file"], doc=table["doc_file"])]

    o.append("CONFIDENCE = {")
    for conf in table["confidence"]:
        o.append(f'    "{conf["id"]}": "{conf["summary"]}",')
    o.append("}")
    o.append("")
    for dev in (ctrl, bms):
        for c in dev.get("constants", []):
            o.append(f"{c['name']} = {c['value']}")
        for f in dev["fields"]:
            if f["count"] > 1:
                o.append(f"{f['count_define']} = {f['count']}")
        for g in dev["groups"]:
            if g.get("c_define"):
                o.append(f"{g['c_define']} = {g['key']}")
    o.append(f"WF_BMS_REG_NEEDED = {reg_needed(bms)}")
    o.append("")
    o.append("")

    for dev in (ctrl, bms):
        o.append(f"{dev['id'].upper()}_FIELDS = [")
        for f in dev["fields"]:
            o.append("    dict(name=%r, unit=%r, device=%r, group=%r, key=%d,"
                     % (f["name"], f["unit"], f["device"], f["group"], f["key"]))
            o.append("         width=%d, count=%d, endian=%r, signed=%r, mask=%r,"
                     % (f["width"], f["count"], f["endian"], f["signed"], f["mask"]))
            o.append("         shift=%d, bias=%d, scale=%r, decimals=%d, confidence=%r),"
                     % (f["shift"], f["bias"], f["scale"], f["decimals"],
                        f["confidence"]))
        o.append("]")
        o.append("")
    o.append("")

    o.append("def _u16le(p, i):")
    o.append("    return p[i] | (p[i + 1] << 8)")
    o.append("")
    o.append("")
    o.append("def _u16be(p, i):")
    o.append("    return (p[i] << 8) | p[i + 1]")
    o.append("")
    o.append("")
    o.append("def _s8(v):")
    o.append("    return v - 0x100 if v >= 0x80 else v")
    o.append("")
    o.append("")
    o.append("def _s16(v):")
    o.append("    return v - 0x10000 if v >= 0x8000 else v")
    o.append("")
    o.append("")

    o.append("def ctrl_apply(live, ftype, payload):")
    o.append('    """Folds one validated Controller frame into the live dict."""')
    o.append("    p = payload")
    first = True
    for g in ctrl["groups"]:
        kw = "if" if first else "elif"
        first = False
        o.append(f"    {kw} ftype == {g['c_define']}:")
        for f in fields_of(ctrl, g["key"]):
            o.append(f'        live["{f["name"]}"] = {py_value(f, "p")}')
        if g.get("valid_flag"):
            o.append(f'        live["{g["valid_flag"]}"] = True')
    o.append("")
    o.append("")

    o.append("def bms_apply(out, reg):")
    o.append('    """Fills out from the registers of one validated 0xd2 response.')
    o.append("")
    o.append("    False, leaving out alone, when the response is too short to")
    o.append('    carry every field the table assigns."""')
    o.append("    if len(reg) < WF_BMS_REG_NEEDED:")
    o.append("        return False")
    for f in bms["fields"]:
        if f["count"] > 1:
            o.append(f'    out["{f["name"]}"] = [{py_value(f, "reg", "i")}'
                     f' for i in range({f["count_define"]})]')
        else:
            o.append(f'    out["{f["name"]}"] = {py_value(f, "reg")}')
    o.append("    return True")
    o.append("")
    o.append("")

    o.append("def ctrl_dump(live, ftype):")
    o.append('    """Every field the given frame type carries, in table order, as')
    o.append("    (name, value, decimals). The C decoder's dump walks the same")
    o.append('    table in the same order, which is what makes them comparable."""')
    first = True
    for g in ctrl["groups"]:
        kw = "if" if first else "elif"
        first = False
        o.append(f"    {kw} ftype == {g['c_define']}:")
        o.append("        return [")
        for f in fields_of(ctrl, g["key"]):
            o.append(f'            ("{f["name"]}", live["{f["name"]}"],'
                     f' {f["decimals"]}),')
        o.append("        ]")
    o.append("    return []")
    o.append("")
    o.append("")
    o.append("def bms_dump(out):")
    o.append('    """Every BMS field, in table order, arrays expanded."""')
    o.append("    rows = []")
    for f in bms["fields"]:
        if f["count"] > 1:
            o.append(f'    for i in range({f["count_define"]}):')
            o.append(f'        rows.append(("{f["name"]}[%d]" % i,'
                     f' out["{f["name"]}"][i], {f["decimals"]}))')
        else:
            o.append(f'    rows.append(("{f["name"]}", out["{f["name"]}"],'
                     f' {f["decimals"]}))')
    o.append("    return rows")

    write(os.path.join(out_dir, "wf_fields.py"), "\n".join(o) + "\n")


# ----------------------------------------------------------- the document

def md_field_table(dev):
    key_head = "Payload byte" if dev["id"] == "controller" else "Register"
    group_head = "Frame type" if dev["id"] == "controller" else "Response"
    rows = ["| %s | %s | Width | Endian | Signed | Mask/shift | Bias/scale | "
            "Unit | Field | Confidence |" % (group_head, key_head),
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for f in dev["fields"]:
        if f["count"] > 1:
            key = f"{f['key']}-{f['key'] + f['count'] - 1}"
        elif f["device"] == "controller" and f["width"] > 1:
            key = f"{f['key']}-{f['key'] + f['width'] - 1}"
        else:
            key = str(f["key"])
        width = ("%d byte%s" % (f["width"], "" if f["width"] == 1 else "s")
                 if f["device"] == "controller" else "u16")
        endian = "-" if (f["device"] == "controller" and f["width"] == 1) \
            else ("big" if f["endian"] == "be" else "little")
        if f["mask"] is not None:
            ms = "`& 0x%02x`" % f["mask"]
            if f["shift"]:
                ms += " `>> %d`" % f["shift"]
        else:
            ms = "-"
        steps = []
        if f["bias"]:
            steps.append("%+d" % f["bias"])
        if f["scale"] != 1:
            steps.append("x %g" % f["scale"])
        scale = ", then ".join(steps) if steps else "-"
        rows.append("| `%s` | %s | %s | %s | %s | %s | %s | %s | `%s` | %s |" % (
            f["group"], key, width, endian, "yes" if f["signed"] else "no", ms,
            scale, f["unit"] or "-", f["name"], f["confidence"]))
    return "\n".join(rows)


def emit_doc(table, path):
    conf_by_id = {c["id"]: c for c in table["confidence"]}
    o = ["# " + table["title"], ""]
    o.append("> Generated from `%s` by `%s`. Do not edit this file: edit the Field"
             % (table["source_file"], table["generator"]))
    o.append("> Table and rebuild. `make test` fails if the two have drifted apart.")
    o.append("")
    for para in table["preamble"]:
        o.append(para)
        o.append("")

    o.append("## Where this comes from")
    o.append("")
    for s in table["sources"]:
        o.append("* [`%s`](%s) - %s" % (s["name"], s["url"], s["note"]))
        o.append("")

    o.append("## Confidence")
    o.append("")
    o.append("| Level | Means | Old label |")
    o.append("| --- | --- | --- |")
    for c in table["confidence"]:
        o.append("| `%s` | %s | %s |" % (c["id"], c["summary"], c["was"]))
    o.append("")
    for c in table["confidence"]:
        o.append("**`%s`** - %s" % (c["id"], c["note"]))
        o.append("")

    for dev in table["devices"]:
        o.append("## " + dev["name"])
        o.append("")
        o.append(dev["note"])
        o.append("")
        if dev["id"] == "controller":
            o.append("| Frame type | Group | What it carries |")
            o.append("| --- | --- | --- |")
            for g in dev["groups"]:
                o.append("| `%s` | %s | %s |" % (g["key"], g["name"], g["note"]))
            o.append("")
        o.append("### Fields")
        o.append("")
        o.append(md_field_table(dev))
        o.append("")
        o.append("Read a value as `(((raw & mask) >> shift) + bias) * scale`, "
                 "sign-extended before the bias where the Signed column says "
                 "so. A `-` in the Bias/scale column means the raw number is "
                 "the value.")
        o.append("")
        o.append("### What each field is, and how far it is trusted")
        o.append("")
        for f in dev["fields"]:
            o.append("**`%s`** (%s) - %s" % (
                f["name"], conf_by_id[f["confidence"]]["summary"], f["note"]))
            o.append("")
        hw = [m for b in dev.get("hand_written", []) for m in b["members"]]
        if hw:
            o.append("### Hand-written, not generated")
            o.append("")
            for m in hw:
                o.append("**`%s`** (`%s`) - %s" % (m["name"], m["ctype"], m["note"]))
                o.append("")
        if dev.get("not_decoded"):
            o.append("### Not decoded")
            o.append("")
            for n in dev["not_decoded"]:
                o.append("**%s** - %s" % (n["name"], n["note"]))
                o.append("")

    for a in table["appendices"]:
        o.append("## " + a["title"])
        o.append("")
        o.append(a["body"])
        o.append("")

    text = "\n".join(o).rstrip() + "\n"
    while "\n\n\n" in text:
        text = text.replace("\n\n\n", "\n\n")
    write(path, text)
    return text


# ------------------------------------------------------------------- main

def write(path, text):
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    old = None
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            old = f.read()
    if old == text:
        return          # keep the mtime, so make does not rebuild the world
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("table", nargs="?", default=DEFAULT_TABLE,
                    help="the Field Table (default: %(default)s)")
    ap.add_argument("--c-dir", help="write wf_fields.h and wf_fields.c here")
    ap.add_argument("--py-dir", help="write wf_fields.py here")
    ap.add_argument("--doc", help="write the field documentation here")
    ap.add_argument("--stamp", help="touch this file when everything is written")
    args = ap.parse_args()

    if not (args.c_dir or args.py_dir or args.doc):
        ap.error("nothing to generate: pass --c-dir, --py-dir or --doc")

    table = load(args.table)
    if args.c_dir:
        emit_c(table, args.c_dir)
    if args.py_dir:
        emit_py(table, args.py_dir)
    if args.doc:
        emit_doc(table, args.doc)
    if args.stamp:
        write(args.stamp, "")
        os.utime(args.stamp, None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
