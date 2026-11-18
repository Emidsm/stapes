// The book. Only the order is here: the design is in template.typ and the text in chapters/.
#import "template.typ": *

#show: book

#cover()

#set page(numbering: "i")
#counter(page).update(1)
#{
  show outline.entry.where(level: 1): it => {
    v(0.7em, weak: true)
    text(font: sans, weight: "semibold", size: 10pt, fill: ink, it)
  }
  heading(level: 1, numbering: none, outlined: false)[Contents]
  outline(title: none, depth: 2)
}

#set page(numbering: "1")
#counter(page).update(1)

#include "chapters/01-overview.typ"
#include "chapters/02-build.typ"
#include "chapters/03-robot.typ"
#include "chapters/04-control.typ"
#include "chapters/05-safety.typ"
#include "chapters/06-faults.typ"
#include "chapters/07-scenarios.typ"
#include "chapters/08-viewer.typ"
#include "chapters/09-verification.typ"
#include "chapters/10-extending.typ"
#include "chapters/11-reference.typ"

#pagebreak(weak: true)
#heading(level: 1, numbering: none)[References]
#bibliography("/references.yml", title: none, style: "ieee")
