# 01 Protocol grammar

Status: **draft for WP0**, spec version `0.1`. Conventions: [00](00_Conventions.md).

The MRS text protocol is one grammar for every object that is stored, sent or written by hand: tasks, conditions, actions, functions, views, requirements, devices, messages, mission headers, timelines and journals. This document defines its **syntax**. The meaning and the slot list of each object code are in specs 02–06.

## 1. Design rules (from the master's thesis §4.1)

1. Human-readable and easy to edit by hand.
2. Every object states its own type and its own data.
3. Structured: an object's sub-objects are separate records that can be read and replaced on their own.
4. **Depth-first order:** a record's sub-records follow it directly, in the order the record references them. Each sub-record is followed by its own sub-records before the next sibling starts.
5. Lists of objects and values are expressible.

## 2. Lexical rules

### 2.1 Characters and whitespace

* A protocol text is **ASCII** (bytes 0x20–0x7E plus tab, CR and LF). Any other byte is an error, except inside a quoted string, where UTF-8 is allowed.
* **Whitespace** is one or more of space, tab, CR, LF. Tokens are separated by whitespace. Whitespace is optional next to `:` and `/`.
* **Comments** start with `#` and run to the end of the line. A comment MAY appear only between records, never inside one. Messages (spec 06) MUST NOT contain comments.

### 2.2 Delimiters

| Character | Meaning |
|---|---|
| `:` | Ends a record header |
| `/` | Ends a record |
| `..` | Range inside a reference token (`A_1..4`) |
| `"` | Starts and ends a quoted string |

### 2.3 Tokens

```ebnf
digit        = "0" | "1" | ... | "9" ;
letter       = "A" | ... | "Z" | "a" | ... | "z" ;
upper        = "A" | ... | "Z" ;
kind         = upper | "?" | "@" ;                        (* see §4.1; never lower case *)

integer      = [ "-" ] digit { digit } ;
number       = [ "-" ] digit { digit } [ "." { digit } ] [ exponent ]
             | [ "-" ] "." digit { digit } [ exponent ] ;
exponent     = ( "e" | "E" ) [ "+" | "-" ] digit { digit } ;
boolean      = "T" | "F" ;

code         = kind "_" subcode ;                          (* object code, e.g. T_A, C_P3, C_? *)
subcode      = ( letter | "?" ) { letter | digit } ;       (* never all digits *)
label        = kind "_" posint ;                           (* local reference, e.g. C_1 *)
range        = kind "_" posint ".." posint ;               (* A_1..4 = A_1 A_2 A_3 A_4 *)
posint       = ( "1" | ... | "9" ) { digit } ;

ident        = ( letter | "_" ) { letter | digit | "_" | "." | "-" } ;
taskid       = issuer "." posint { "." posint }            (* issuer.seq[.child...], see spec 06 §3 *)
             | "0" ;                                       (* unassigned: inside a tree, set by the decomposer *)
issuer       = ( letter | "_" ) { letter | digit | "_" | "-" } ;   (* no dots *)
string       = '"' { strchar | '\"' | '\\' } '"' ;         (* strchar: any char except " and \ *)
```

Disambiguation, applied in this order to a whitespace-separated token:

1. `T` or `F` alone is a **boolean**.
2. A token matching `range` is a **range**; one matching `label` is a **label**.
3. A token matching `code` is a **code**.
4. A token matching `number` is a **number** (an `integer` is also a number).
5. A token matching `taskid` with a `.` is a **task id**. (Worldview field paths never match, because their segments start with a letter; spec 05 §2.)
6. A token matching `ident` is an **identifier**.
7. A token starting with `"` is a **string**.

Anything else is a lexical error.

*Note:* `C_1` is a label and `C_N` is a code because a subcode never starts with a digit. `m_fl` is an identifier, not a code, because a kind is never lower case. Identifiers SHOULD start with a lower-case letter so they never look like a code or a label.

Numbers:
* Integers MUST fit in a signed 64-bit integer.
* Other numbers are parsed as IEEE-754 binary64. A writer MUST write them so that they parse back to the same binary64 value (shortest round-trip form, for example `std::to_chars`).
* `nan`, `inf`, hexadecimal and leading `+` are not valid. The words `nan`, `inf` and `infinity`, in any case, are a lexical error even though they look like identifiers.

## 3. Records

### 3.1 Structure

```ebnf
text         = { record | comment } ;
record       = header ":" body "/" ;
header       = kind                                        (* top-level record *)
             | label ;                                     (* sub-record *)
body         = code { field }                              (* most kinds *)
             | literal-body ;                              (* kinds without a code, §4.1 *)
field        = number | boolean | label | range | ident | taskid | string | code ;
```

* A **top-level record** has a header made of its kind letter alone (`T:`). Its body starts with a code of the same kind (`T_A`).
* A **sub-record** has a header that is a label (`C_1:`). Its body starts with a code of the label's kind (`C_P3`).
* The **fields** after the code are that code's slots, in the order its spec defines. Slots are positional. A code's spec says which slots are optional; an optional slot is recognised by its kind (for example an optional `R_` label), never by position alone.

### 3.2 References and the depth-first rule

* A label or range in a body is a **reference** to a sub-record of that record.
* The referenced sub-records MUST follow the record **immediately**, in the **same order** as the references, each one followed by its own sub-records (pre-order traversal).
* The header of each sub-record MUST equal the reference that points to it. A parser MUST check this and report a mismatch.
* **Labels are local to their parent.** The same label MAY be used again under a different parent. Two references in one body MUST NOT use the same label.
* References form a **tree**. A sub-record has exactly one parent. Sharing a sub-record between two parents is not expressible; it has to be repeated.
* A range `K_a..b` with `a ≤ b` expands to `K_a K_(a+1) … K_b` before the slots are read, so one range MAY fill several slots (`T_B ... C_1..3` gives the start, end and until conditions). `a > b` is an error.

Example. The parent references `C_1 C_2 A_1..2`, so the order of the following records is fixed:

```
T: T_A op.1 1.0 0 C_1 C_2 A_1..2/
C_1: C_P3 10 20 15 1 0.5 0 -1/
C_2: C_N/
A_1: A_L 1 65280/
A_2: A_N/
```

A nested example: `C_1` is a logic condition with its own sub-records, which come before `C_2`:

```
T: T_A op.2 1.0 0 C_1 C_2 A_1/
C_1: C_L 1 C_1 C_2/
C_1: C_? ?_1/
?_1: airborne T/
C_2: C_H 15 0.5/
C_2: C_N/
A_1: A_N/
```

### 3.3 Records without a code (literal bodies)

Some kinds carry plain data and have no code. Their bodies are defined here:

```ebnf
predicate-body = ident boolean ;                     (* kind "?": a named boolean, e.g. "?_1: airborne T/" *)
version-body   = "MRS" number ;                      (* kind "@": "@: MRS 0.1/" *)
```

### 3.4 Files

* A **file** is a sequence of top-level records, each with its sub-records, plus comments between records.
* A file SHOULD start with a version record `@: MRS 0.1/`. If it does not, version `0.1` is assumed.
* File extensions:

| Extension | Content |
|---|---|
| `.mrst` | Tasks (task sets, task libraries) |
| `.mrsb` | Behaviour library (behaviour tasks plus the conditions they fulfil, spec 03 §7) |
| `.mrsd` | Robot definition (device tree, spec 04) |
| `.mrsl` | Timeline: task dispatch schedule (spec 06 §7) |
| `.mrsj` | Task journal (spec 06 §8) |
| `.mrsm` | Message log: one `M` record per line (spec 06 §2) |
| `.mrsp` | Port map: one `P_A` record per assigned port requirement (spec 04 §6.2) |

### 3.5 Writer rules (canonical form)

A writer MUST produce the **canonical form**, so that write → parse → write gives identical bytes:

* One record per line. The header is followed by `: ` (colon, space). Fields are separated by one space. A record ends with `/` and a newline.
* Labels are numbered 1, 2, … per kind within each parent, in reference order. Consecutive labels of one kind are written as a range when there are three or more (`A_1..4`), and listed otherwise.
* Numbers use the shortest round-trip form. Integers are written without a decimal point.
* No comments and no blank lines, except one blank line between top-level records in a file.
* **Message form** (spec 06 §2, and `.mrsm` message logs): each top-level record is written with all its sub-records on one line, separated by nothing, followed by a newline. There are no blank lines.

A parser MUST accept any valid input, not only the canonical form.

## 4. Kinds and codes

### 4.1 Kinds

| Kind | Object | Has a code | Defined in |
|---|---|---|---|
| `@` | Version | no | §3.3 |
| `T` | Task | yes | spec 03 |
| `C` | Condition | yes | spec 03 |
| `?` | Predicate literal | no | §3.3 |
| `R` | Requirement | yes | spec 03 |
| `A` | Action | yes | spec 02 |
| `F` | Function | yes | spec 02 |
| `V` | View | yes | spec 05 |
| `D` | Device node | yes | spec 04 |
| `P` | Port requirement (`P_R`) or port map entry (`P_A`) | yes | spec 04 |
| `K` | Capability | yes | spec 04 |
| `M` | Message | yes | spec 06 |
| `H` | Mission header | yes | spec 06 |
| `L` | Timeline entry | yes | spec 06 |
| `J` | Journal entry | yes | spec 06 |
| `B` | Behaviour-library entry | yes | spec 03 §7 |

All other letters are **reserved**.

### 4.2 Code index

The full list of codes, with links to their slot definitions:

| Kind | Codes |
|---|---|
| `T` | `T_A`, `T_B`, `T_P`, `T_S`, `T_L`, `T_O`; reserved `T_D`, `T_G`, `T_U` (spec 03 §2) |
| `C` | `C_N`, `C_F`, `C_?`, `C_m`, `C_T`, `C_W`, `C_L`, `C_V`, `C_S`, `C_P`, `C_P3`, `C_G`, `C_H`, `C_Q` (spec 03 §4) |
| `R` | `R_S`, `R_F`, `R_A`, `R_R`, `R_E`, `R_K`, `R_C` (spec 03 §5) |
| `A` | see the action registry (spec 02 §4) |
| `F` | `F_K`, `F_L`, `F_X`, `F_S`, `F_P`, `F_C`; reserved `F_E` (spec 02 §7) |
| `V` | see the view registry (spec 05 §6) |
| `D`, `P`, `K` | spec 04 |
| `M`, `H`, `L`, `J` | spec 06 |
| `B` | `B_E` (spec 03 §7) |

## 5. Errors

A parser MUST report, with the byte offset and the record header:

| Error | Example |
|---|---|
| Lexical error | an unknown character, a malformed number |
| Unknown or reserved code | `T_Z`, `T_G` |
| Slot error | wrong number of slots, wrong slot type, missing required slot |
| Reference error | a sub-record is missing, out of order, or its header does not match; a label is used twice in one body |
| Version error | an unsupported MAJOR version |
| Trailing data | text after the last complete record that is not a comment or whitespace |

The **offset** is the first byte of the offending token, counted from the start of the text, with these cases:
* a missing slot: the closing `/` of the record;
* a missing or misplaced sub-record: the header of the record found in its place, or the end of the text;
* a check that needs a sub-record's code (for example a nested `A_MAP`): that sub-record's code;
* trailing data: the start of the unterminated record.

A parser MUST NOT guess or repair input. A runtime that receives a malformed message drops it and counts the error (spec 06 §2).

## 6. Complete EBNF (summary)

```ebnf
text           = { ws | comment | record } ;
comment        = "#" { any-char-except-newline } newline ;
record         = header ws? ":" ws? body ws? "/" ;
header         = kind | label ;
body           = code { ws field } | predicate-body | version-body ;
field          = number | boolean | label | range | ident | taskid | string | code ;
code           = kind "_" subcode ;
subcode        = ( letter | "?" ) { letter | digit } ;
label          = kind "_" posint ;
range          = kind "_" posint ".." posint ;
predicate-body = ident ws boolean ;
version-body   = "MRS" ws number ;
```

The slot list for each code is the **second level** of the grammar. It is given as a table per code in specs 02–06, with these slot types:

| Slot type | Token |
|---|---|
| `int` | integer |
| `num` | number |
| `bool` | boolean |
| `id` | identifier |
| `tid` | task id; in a `tid` slot a plain integer `n` means issuer `local`, and `0` means unassigned |
| `str` | quoted string |
| `code` | an object code |
| `K` | one label of kind K |
| `K*` | zero or more labels or ranges of kind K |
| `K+` | one or more labels or ranges of kind K |
| `[x]` | optional slot x |
| `x*` | zero or more of x |
