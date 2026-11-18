// The manual's design, written once. The text lives in chapters/*.typ.
//
// Compiled with `--root manual --font-path manual/fonts --ignore-system-fonts`, so the PDF does
// not depend on what is installed on the machine that builds it: see tools/build_manual.py.
//
// Colours are not written here. They come from tokens.json, the same values the plots and the
// viewer use. Signal red means only a safety stop or an injected fault.

#let tokens = json("/tokens.json")
#let ink = rgb(tokens.at("ink"))
#let teal = rgb(tokens.at("teal"))
#let pale-teal = rgb(tokens.at("pale-teal"))
#let signal-red = rgb(tokens.at("signal-red"))
#let muted = rgb(tokens.at("muted"))
#let rule = rgb(tokens.at("rule"))
#let paper = rgb(tokens.at("paper"))

#let serif = "Libertinus Serif"
#let sans = "Inter"
#let mono = "DejaVu Sans Mono"

// Everything measured comes from this file, which tools/build_manual.py writes from the test
// results. No measured number is typed into a chapter.
#let data = json("/build/data.json")
#let commit = sys.inputs.at("commit", default: "development")
#let built-on = sys.inputs.at("date", default: "undated")

// A measured value by its key in the test results, with a fixed number of decimals.
#let measured(key, digits: 0) = {
  assert(key in data.measurements, message: "no test recorded the measurement " + key)
  str(calc.round(float(data.measurements.at(key)), digits: digits))
}

// ── Callouts ────────────────────────────────────────────────────────────────
#let _callout(label, bar, fill, label-colour, body) = block(
  width: 100%,
  fill: fill,
  stroke: (left: 2.6pt + bar),
  radius: (right: 3pt),
  inset: (left: 11pt, right: 11pt, y: 9pt),
  above: 1.2em,
  below: 1.2em,
  breakable: false,
)[
  #set par(justify: false, first-line-indent: 0pt)
  #text(font: sans, weight: "bold", size: 7.4pt, tracking: 0.11em, fill: label-colour, upper(label))
  #v(2pt)
  #set text(size: 9.8pt)
  #body
]
#let note(body) = _callout("Note", muted, paper, muted, body)
#let why(body) = _callout("Why", teal, pale-teal.lighten(55%), teal, body)
// Red is reserved: this callout is only for text about a safety stop or an injected fault.
#let stop-note(body) = _callout("Safety stop", signal-red, signal-red.lighten(92%), signal-red, body)

// ── A numbered procedure ────────────────────────────────────────────────────
#let procedure(title, steps) = block(width: 100%, above: 1.2em, below: 1.2em)[
  #block(
    width: 100%,
    fill: teal,
    radius: (top: 3pt),
    inset: (x: 11pt, y: 6pt),
    sticky: true,
    text(font: sans, weight: "semibold", size: 9pt, fill: white, title),
  )
  #block(
    width: 100%,
    stroke: (x: 0.7pt + rule, bottom: 0.7pt + rule),
    radius: (bottom: 3pt),
    inset: (x: 11pt, y: 9pt),
  )[
    #set enum(
      numbering: n => text(font: sans, weight: "bold", size: 8.6pt, fill: teal)[#n],
      body-indent: 0.7em,
      spacing: 0.85em,
    )
    #set par(justify: false)
    #steps
  ]
]

// ── Tables ──────────────────────────────────────────────────────────────────
#let data-table(columns, ..cells) = block(above: 1.1em, below: 1.1em, {
  set text(font: sans, size: 8.6pt)
  set par(justify: false)
  table(
    columns: columns,
    stroke: (x, y) => (
      top: if y == 0 { 1pt + ink } else if y == 1 { 0.6pt + ink } else { 0.4pt + rule },
      bottom: 0.4pt + rule,
    ),
    inset: (x: 6pt, y: 5.5pt),
    align: left + top,
    fill: (x, y) => if y == 0 { paper } else { none },
    ..cells.pos().enumerate().map(((i, cell)) => if i < columns.len() { strong(cell) } else { cell }),
  )
})

// ── Figures from the build ──────────────────────────────────────────────────
#let plot(name, caption, width: 100%) = figure(
  image("/build/figures/" + name + ".svg", width: width),
  caption: caption,
)

// ── Chapters ────────────────────────────────────────────────────────────────
#let chapter(title, summary: none, tag: none) = {
  pagebreak(weak: true)
  // Figures and tables are numbered within their chapter.
  counter(figure.where(kind: image)).update(0)
  counter(figure.where(kind: table)).update(0)
  // The label goes on the heading itself, so that `@tag` refers to the chapter.
  [#heading(level: 1, supplement: [Chapter], title)#if tag != none { label(tag) }]
  if summary != none {
    block(width: 88%, above: 0.4em, below: 1.8em,
      text(font: serif, style: "italic", size: 11.5pt, fill: muted, summary))
  }
}

#let cover() = page(
  fill: ink,
  margin: (x: 2.6cm, top: 7cm, bottom: 2.6cm),
  header: none,
  footer: none,
)[
  #set text(fill: white)
  #set par(justify: false)
  #text(font: sans, weight: "semibold", size: 10pt, tracking: 0.22em, fill: pale-teal)[OPERATION MANUAL]
  #v(14pt)
  #text(font: sans, weight: "bold", size: 54pt, tracking: -0.02em)[Stapes]
  #v(10pt)
  #block(width: 80%, text(font: serif, size: 15pt, fill: pale-teal)[
    A deterministic simulation environment for a 6-DOF surgical robot arm
  ])
  #v(1fr)
  #line(length: 100%, stroke: 0.6pt + teal)
  #v(8pt)
  #text(font: sans, size: 9pt, fill: pale-teal)[
    Version #data.version #h(1fr) Built #built-on from commit #commit
  ]
  #v(4pt)
  #text(font: sans, size: 8pt, fill: muted.lighten(40%))[
    An engineering project, not a medical device. The arm, the anatomy and the loads are
    illustrative. Not affiliated with any manufacturer.
  ]
]

#let book(body) = {
  set document(title: "Stapes: operation manual", author: "Emiliano De Santiago Morales")
  set page(
    paper: "a4",
    margin: (inside: 2.7cm, outside: 2.3cm, top: 2.6cm, bottom: 2.6cm),
    header: context {
      if counter(page).get().first() > 1 {
        set text(font: sans, size: 7.6pt, fill: muted)
        let chapters = query(heading.where(level: 1).before(here()))
        [Stapes operation manual #h(1fr) #if chapters.len() > 0 { chapters.last().body }]
        v(-6pt)
        line(length: 100%, stroke: 0.4pt + rule)
      }
    },
    footer: context {
      set text(font: sans, size: 8pt, fill: muted)
      align(center, counter(page).display())
    },
  )
  set text(font: serif, size: 10.6pt, fill: ink, lang: "en")
  set par(justify: true, leading: 0.68em, spacing: 1em)

  set heading(numbering: "1.1")
  show heading.where(level: 1): it => block(above: 0pt, below: 1.1em, {
    // Contents and References are headings too, and they are not chapters.
    if it.numbering != none {
      text(font: sans, weight: "semibold", size: 9pt, tracking: 0.16em, fill: teal,
        upper[Chapter #counter(heading).display("1")])
      v(4pt)
    }
    text(font: sans, weight: "bold", size: 25pt, tracking: -0.01em, fill: ink, it.body)
    v(4pt)
    line(length: 3.2cm, stroke: 1.6pt + teal)
  })
  show heading.where(level: 2): it => block(above: 1.6em, below: 0.7em,
    text(font: sans, weight: "bold", size: 13pt, fill: ink)[
      #text(fill: teal, counter(heading).display("1.1")) #h(0.4em) #it.body
    ])
  show heading.where(level: 3): it => block(above: 1.2em, below: 0.5em,
    text(font: sans, weight: "semibold", size: 10.4pt, fill: ink, it.body))

  show raw: set text(font: mono)
  show raw.where(block: false): set text(size: 1.12em)
  // A shell command is light on ink; code and output are dark on paper.
  show raw.where(block: true): it => {
    let is-command = it.lang == "sh"
    set text(size: 8.2pt, fill: if is-command { white } else { ink })
    block(
      width: 100%,
      fill: if is-command { ink } else { paper },
      stroke: if is-command { none } else { 0.5pt + rule },
      radius: 3pt,
      inset: (x: 10pt, y: 8pt),
      above: 1em,
      below: 1em,
      breakable: false,
      // Plain text: syntax colours would bring in hues the palette does not have.
      it.text,
    )
  }
  show link: it => text(fill: teal, it)

  // Figures are numbered chapter.figure.
  set figure(numbering: n => numbering("1.1", counter(heading).get().first(), n), gap: 8pt)
  show figure.caption: it => {
    set text(font: sans, size: 8.4pt, fill: muted)
    set par(justify: false)
    [#text(weight: "semibold", fill: teal)[#it.supplement #it.counter.display(it.numbering)] #h(0.4em) #it.body]
  }
  set list(marker: text(fill: teal)[▪], body-indent: 0.6em)

  body
}
