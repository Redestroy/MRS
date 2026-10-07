#include "mrs/protocol/Parser.h"

#include <cctype>
#include <charconv>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "mrs/device/Action.h"

namespace MRS {
	namespace Protocol {
		const char* ErrorClassName(ErrorClass c) {
			switch (c) {
			case ErrorClass::Lexical: return "lexical_error";
			case ErrorClass::Code: return "code_error";
			case ErrorClass::Slot: return "slot_error";
			case ErrorClass::Reference: return "reference_error";
			case ErrorClass::Version: return "version_error";
			case ErrorClass::Trailing: return "trailing_data";
			}
			return "unknown";
		}

		namespace {
			// ---------------------------------------------------------------- lexer (spec 01 §2)

			enum class Tok { Bool, Range, Label, Code, Int, Num, TaskId, Ident, Str };

			struct Token {
				Tok type;
				std::string_view text;
				std::size_t offset;
				std::string str;      // unescaped value of a string
				char kind = 0;        // Label, Range, Code
				std::int64_t a = 0;   // Label number, Range start
				std::int64_t b = 0;   // Range end
			};

			struct Failure {
				ParseError error;
			};

			[[noreturn]] void Fail(ErrorClass c, std::size_t offset, std::string header, std::string message) {
				throw Failure{ParseError{c, offset, std::move(header), std::move(message)}};
			}

			bool IsLetter(char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); }
			bool IsDigit(char c) { return c >= '0' && c <= '9'; }
			bool IsKind(char c) { return (c >= 'A' && c <= 'Z') || c == '?' || c == '@'; }
			bool IsSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

			// posint = "1".."9" { digit }
			bool ParsePosInt(std::string_view s, std::int64_t& out) {
				if (s.empty() || s[0] == '0') return false;
				for (char c : s)
					if (!IsDigit(c)) return false;
				auto res = std::from_chars(s.data(), s.data() + s.size(), out);
				return res.ec == std::errc() && res.ptr == s.data() + s.size();
			}

			bool MatchLabel(std::string_view s, char& kind, std::int64_t& n) {
				if (s.size() < 3 || !IsKind(s[0]) || s[1] != '_') return false;
				kind = s[0];
				return ParsePosInt(s.substr(2), n);
			}

			bool MatchRange(std::string_view s, char& kind, std::int64_t& a, std::int64_t& b) {
				if (s.size() < 6 || !IsKind(s[0]) || s[1] != '_') return false;
				auto dots = s.find("..");
				if (dots == std::string_view::npos) return false;
				kind = s[0];
				return ParsePosInt(s.substr(2, dots - 2), a) && ParsePosInt(s.substr(dots + 2), b);
			}

			bool MatchCode(std::string_view s) {
				if (s.size() < 3 || !IsKind(s[0]) || s[1] != '_') return false;
				if (!(IsLetter(s[2]) || s[2] == '?')) return false;
				for (std::size_t i = 3; i < s.size(); ++i)
					if (!IsLetter(s[i]) && !IsDigit(s[i])) return false;
				return true;
			}

			// Returns 0 = not a number, 1 = integer, 2 = number.
			int MatchNumber(std::string_view s) {
				std::size_t i = 0;
				if (i < s.size() && s[i] == '-') ++i;
				std::size_t int_digits = 0, frac_digits = 0;
				while (i < s.size() && IsDigit(s[i])) ++i, ++int_digits;
				bool dot = false, exp = false;
				if (i < s.size() && s[i] == '.') {
					dot = true;
					++i;
					while (i < s.size() && IsDigit(s[i])) ++i, ++frac_digits;
				}
				if (int_digits == 0 && frac_digits == 0) return 0;
				if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
					exp = true;
					++i;
					if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
					std::size_t e = 0;
					while (i < s.size() && IsDigit(s[i])) ++i, ++e;
					if (e == 0) return 0;
				}
				if (i != s.size()) return 0;
				return (dot || exp) ? 2 : 1;
			}

			bool IsIssuerChar(char c) { return IsLetter(c) || IsDigit(c) || c == '_' || c == '-'; }

			// taskid with a dot: issuer "." posint { "." posint }
			bool MatchTaskId(std::string_view s) {
				auto dot = s.find('.');
				if (dot == std::string_view::npos || dot == 0) return false;
				if (!(IsLetter(s[0]) || s[0] == '_')) return false;
				for (std::size_t i = 1; i < dot; ++i)
					if (!IsIssuerChar(s[i])) return false;
				std::string_view rest = s.substr(dot + 1);
				while (true) {
					auto next = rest.find('.');
					std::int64_t n;
					if (!ParsePosInt(rest.substr(0, next), n)) return false;
					if (next == std::string_view::npos) return true;
					rest = rest.substr(next + 1);
				}
			}

			bool MatchIdent(std::string_view s) {
				if (s.empty() || !(IsLetter(s[0]) || s[0] == '_')) return false;
				for (char c : s)
					if (!(IsLetter(c) || IsDigit(c) || c == '_' || c == '.' || c == '-')) return false;
				return true;
			}

			bool IsNonFiniteWord(std::string_view s) {
				std::string lower;
				for (char c : s) lower += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
				return lower == "nan" || lower == "inf" || lower == "infinity";
			}

			// Classifies one whitespace-separated token in the order of spec 01 §2.
			Token Classify(std::string_view s, std::size_t offset, const std::string& header) {
				Token t{Tok::Ident, s, offset, {}};
				if (s == "T" || s == "F") {
					t.type = Tok::Bool;
					return t;
				}
				if (MatchRange(s, t.kind, t.a, t.b)) {
					t.type = Tok::Range;
					return t;
				}
				if (MatchLabel(s, t.kind, t.a)) {
					t.type = Tok::Label;
					return t;
				}
				if (MatchCode(s)) {
					t.type = Tok::Code;
					t.kind = s[0];
					return t;
				}
				if (int n = MatchNumber(s)) {
					t.type = n == 1 ? Tok::Int : Tok::Num;
					if (n == 1) {
						std::int64_t v;
						auto res = std::from_chars(s.data(), s.data() + s.size(), v);
						if (res.ec != std::errc()) Fail(ErrorClass::Lexical, offset, header, "integer outside the 64-bit range");
					} else {
						double v;
						auto res = std::from_chars(s.data(), s.data() + s.size(), v);
						if (res.ec != std::errc() || res.ptr != s.data() + s.size())
							Fail(ErrorClass::Lexical, offset, header, "malformed or out-of-range number");
					}
					return t;
				}
				if (MatchTaskId(s)) {
					t.type = Tok::TaskId;
					return t;
				}
				if (MatchIdent(s)) {
					if (IsNonFiniteWord(s)) Fail(ErrorClass::Lexical, offset, header, "nan and inf are not valid numbers");
					t.type = Tok::Ident;
					return t;
				}
				Fail(ErrorClass::Lexical, offset, header, "invalid token '" + std::string(s) + "'");
			}

			struct RawRecord {
				std::size_t offset;          // header start
				std::string header;
				std::size_t end_offset;      // offset of the closing '/'
				std::vector<Token> tokens;
			};

			std::vector<RawRecord> Lex(std::string_view text) {
				std::vector<RawRecord> out;
				std::size_t i = 0;
				const std::size_t n = text.size();
				while (true) {
					while (i < n && IsSpace(text[i])) ++i;
					if (i >= n) break;
					if (text[i] == '#') {
						while (i < n && text[i] != '\n') ++i;
						continue;
					}
					RawRecord rec;
					rec.offset = i;
					std::size_t j = i;
					while (j < n && text[j] != ':' && text[j] != '/' && text[j] != '\n') ++j;
					if (j >= n) Fail(ErrorClass::Trailing, rec.offset, "", "unterminated record");
					if (text[j] != ':') Fail(ErrorClass::Lexical, rec.offset, "", "record header without ':'");
					std::string_view header = text.substr(i, j - i);
					while (!header.empty() && IsSpace(header.back())) header.remove_suffix(1);
					rec.header = std::string(header);
					char kind;
					std::int64_t num;
					const bool ok = (header.size() == 1 && IsKind(header[0])) || MatchLabel(header, kind, num);
					if (!ok) Fail(ErrorClass::Lexical, rec.offset, rec.header, "invalid record header");
					i = j + 1;
					// Body tokens up to the closing '/' outside strings.
					while (true) {
						while (i < n && IsSpace(text[i])) ++i;
						if (i >= n) Fail(ErrorClass::Trailing, rec.offset, rec.header, "unterminated record");
						if (text[i] == '/') {
							rec.end_offset = i;
							++i;
							break;
						}
						const std::size_t start = i;
						if (text[i] == '"') {
							std::string value;
							++i;
							while (true) {
								if (i >= n) Fail(ErrorClass::Trailing, rec.offset, rec.header, "unterminated string");
								char c = text[i];
								if (c == '"') break;
								if (c == '\\') {
									if (i + 1 < n && (text[i + 1] == '"' || text[i + 1] == '\\')) {
										value += text[i + 1];
										i += 2;
										continue;
									}
									Fail(ErrorClass::Lexical, i, rec.header, "invalid escape in string");
								}
								value += c;
								++i;
							}
							++i;  // closing quote
							if (i < n && !IsSpace(text[i]) && text[i] != '/')
								Fail(ErrorClass::Lexical, i, rec.header, "text directly after a string");
							Token t{Tok::Str, text.substr(start, i - start), start, {}};
							t.str = std::move(value);
							rec.tokens.push_back(std::move(t));
							continue;
						}
						while (i < n && !IsSpace(text[i]) && text[i] != '/') ++i;
						rec.tokens.push_back(Classify(text.substr(start, i - start), start, rec.header));
					}
					out.push_back(std::move(rec));
				}
				return out;
			}

			// ---------------------------------------------------------------- slot cursor

			// A code of the version 0.1 schema (defined after the schema).
			bool IsSchemaCode(const std::string& code);

			class Cursor {
			public:
				// Ranges are expanded into single labels first, because a canonical range
				// can span several slots (T_B C_1..3 is start, end and until).
				Cursor(const RawRecord& raw, Record& rec) : raw_(raw), rec_(rec), i_(0) {
					for (std::size_t k = 1; k < raw.tokens.size(); ++k) {
						const Token& t = raw.tokens[k];
						if (t.type != Tok::Range) {
							tokens_.push_back(t);
							continue;
						}
						if (t.a > t.b) Fail(ErrorClass::Reference, t.offset, raw.header, "reversed range");
						if (t.b - t.a >= 4096) Fail(ErrorClass::Reference, t.offset, raw.header, "range too long");
						for (std::int64_t n = t.a; n <= t.b; ++n) {
							Token label = t;
							label.type = Tok::Label;
							label.a = n;
							tokens_.push_back(label);
						}
					}
				}

				const Token* Peek() const { return i_ < tokens_.size() ? &tokens_[i_] : nullptr; }
				bool AtEnd() const { return i_ >= tokens_.size(); }

				[[noreturn]] void SlotError(const std::string& msg) const {
					const Token* t = Peek();
					Fail(ErrorClass::Slot, t ? t->offset : raw_.end_offset, raw_.header, rec_.code + ": " + msg);
				}

				std::int64_t Int() {
					const Token* t = Peek();
					if (!t || t->type != Tok::Int) SlotError("expected an integer");
					std::int64_t v;
					std::from_chars(t->text.data(), t->text.data() + t->text.size(), v);
					Push(Field::MakeInt(v));
					return v;
				}

				std::int64_t Count() {
					const Token* t = Peek();
					std::int64_t v = Int();
					if (v < 0) Fail(ErrorClass::Slot, t->offset, raw_.header, rec_.code + ": negative count");
					return v;
				}

				double Num() {
					const Token* t = Peek();
					if (!t || (t->type != Tok::Int && t->type != Tok::Num)) SlotError("expected a number");
					double v;
					std::from_chars(t->text.data(), t->text.data() + t->text.size(), v);
					Push(Field::MakeNum(v));
					return v;
				}

				void Nums(int count) {
					for (int k = 0; k < count; ++k) Num();
				}

				void Bool() {
					const Token* t = Peek();
					if (!t || t->type != Tok::Bool) SlotError("expected T or F");
					Push(Field::MakeBool(t->text == "T"));
				}

				std::string Id() {
					const Token* t = Peek();
					if (!t || t->type != Tok::Ident) SlotError("expected an identifier");
					Push(Field::MakeText(FieldType::Id, std::string(t->text)));
					return std::string(t->text);
				}

				void IdIn(const std::set<std::string>& allowed) {
					const Token* t = Peek();
					std::string v = Id();
					if (!allowed.count(v)) Fail(ErrorClass::Slot, t->offset, raw_.header, rec_.code + ": '" + v + "' is not an allowed value");
				}

				void IdOrStr() {
					const Token* t = Peek();
					if (t && t->type == Tok::Str) {
						Push(Field::MakeText(FieldType::Str, t->str));
						return;
					}
					Id();
				}

				// Task id; a plain integer n >= 0 is also accepted (spec 01 §6).
				void Tid() {
					const Token* t = Peek();
					if (!t || !(t->type == Tok::TaskId || (t->type == Tok::Int && t->text[0] != '-')))
						SlotError("expected a task id");
					Push(Field::MakeText(FieldType::TaskId, std::string(t->text)));
				}

				// A code of the given kind, which must be valid where `valid` says so.
				std::string Code(char kind, const std::function<bool(const std::string&)>& valid) {
					const Token* t = Peek();
					if (!t || t->type != Tok::Code || t->kind != kind) SlotError(std::string("expected a ") + kind + " code");
					std::string code(t->text);
					if (!valid(code)) Fail(ErrorClass::Code, t->offset, raw_.header, "code " + code + " is not allowed here");
					Push(Field::MakeText(FieldType::Code, code));
					return code;
				}

				// Number, boolean, identifier, code or string (parameter values, spec 04 §2.1).
				void Value() {
					const Token* t = Peek();
					if (!t) SlotError("missing value");
					switch (t->type) {
					case Tok::Int:
					case Tok::Num: Num(); return;
					case Tok::Bool: Bool(); return;
					case Tok::Ident: Id(); return;
					case Tok::Str: IdOrStr(); return;
					case Tok::Code: {
						// A code value names an object code, so it must be one (spec 01 §2, spec 04 §2.1).
						std::string code(t->text);
						if (!IsSchemaCode(code)) Fail(ErrorClass::Code, t->offset, raw_.header, "unknown code " + code + " as a value");
						Push(Field::MakeText(FieldType::Code, code));
						return;
					}
					default: SlotError("expected a number, boolean, identifier, code or string");
					}
				}

				void Params() {
					std::int64_t count = Count();
					for (std::int64_t k = 0; k < count; ++k) {
						Id();
						Value();
					}
				}

				// Labels or ranges of the given kinds; returns the number of references.
				std::size_t Labels(const std::string& kinds, std::size_t min = 0, std::size_t max = std::numeric_limits<std::size_t>::max()) {
					std::size_t count = 0;
					while (count < max) {
						const Token* t = Peek();
						if (!t || t->type != Tok::Label || kinds.find(t->kind) == std::string::npos) break;
						auto key = std::make_pair(t->kind, t->a);
						if (!seen_.insert(key).second) Fail(ErrorClass::Reference, t->offset, raw_.header, "label used twice in one body");
						Field f = Field::MakeRef(refs.size());
						f.offset = t->offset;
						rec_.fields.push_back(f);
						refs.push_back(std::string(1, t->kind) + "_" + std::to_string(t->a));
						++count;
						++i_;
					}
					if (count < min) SlotError("expected at least " + std::to_string(min) + " " + kinds + " references");
					return count;
				}

				void Label(char kind) { Labels(std::string(1, kind), 1, 1); }

				void End() {
					if (!AtEnd()) SlotError("too many slots");
				}

				std::vector<std::string> refs;  // expected sub-record headers, in order

			private:
				void Push(Field f) {
					f.offset = tokens_[i_].offset;
					rec_.fields.push_back(std::move(f));
					++i_;
				}

				const RawRecord& raw_;
				Record& rec_;
				std::size_t i_;
				std::vector<Token> tokens_;
				std::set<std::pair<char, std::int64_t>> seen_;
			};

			// ---------------------------------------------------------------- code schema (specs 02-06)

			using Slots = std::function<void(Cursor&, const std::string& code)>;

			const std::set<std::string> kPoolStates = {"AVAILABLE", "BLOCKED", "CLAIMED", "ACTIVE", "DONE",
			                                            "FAILED", "DUMPED_SELF", "IMPOSSIBLE", "CANCELLED"};

			bool IsLeafAction(const std::string& code) { return Device::FindAction(code).has_value(); }

			struct Schema {
				std::map<std::string, Slots> codes;
				std::set<std::string> reserved;

				bool Known(const std::string& code) const { return codes.count(code) > 0; }
			};

			const Schema& GetSchema();

			bool IsKnown(const std::string& code) { return GetSchema().Known(code); }

			void TaskSlots(Cursor& c, const std::string& code) {
				c.Tid();
				const Token* prio = c.Peek();
				if (c.Num() <= 0) Fail(ErrorClass::Slot, prio->offset, "", code + ": priority must be > 0");
				const Token* state = c.Peek();
				std::int64_t s = c.Int();
				if (s < 0 || s > 6) Fail(ErrorClass::Slot, state->offset, "", code + ": state must be 0-6");
				if (code == "T_L") c.Count();
				c.Label('C');
				c.Label('C');
				if (code == "T_B") c.Label('C');
				c.Labels("R", 0, 1);
				if (code == "T_A" || code == "T_P") c.Labels("A", 1);
				else if (code == "T_B") c.Label('T');
				else c.Labels("T", 1);
			}

			void ConditionSlots(Cursor& c, const std::string& code) {
				if (code == "C_N" || code == "C_F") return;
				if (code == "C_?") return c.Label('?');
				if (code == "C_m") {
					c.Id();
					c.IdIn({"lt", "le", "eq", "ne", "ge", "gt"});
					c.Nums(2);
					return;
				}
				if (code == "C_T") {
					c.Int();
					return;
				}
				if (code == "C_W") {
					c.Num();
					return;
				}
				if (code == "C_L") {
					const Token* t = c.Peek();
					std::int64_t op = c.Int();
					if (op < 1 || op > 7) Fail(ErrorClass::Slot, t->offset, "", "C_L: op must be a LogicalOperation 1-7");
					if (op == 3) c.Label('C');
					else c.Labels("C", 2);
					return;
				}
				if (code == "C_V") return c.Label('V');
				if (code == "C_S") {
					c.Tid();
					c.IdIn(kPoolStates);
					return;
				}
				if (code == "C_P") return c.Nums(5);
				if (code == "C_P3") return c.Nums(7);
				if (code == "C_G") return c.Nums(5);
				if (code == "C_H") return c.Nums(2);
			}

			void RequirementSlots(Cursor& c, const std::string& code) {
				if (code == "R_S") {
					c.Labels("R", 1);
				} else if (code == "R_F") {
					c.Num();
					do c.Id();
					while (!c.AtEnd());
				} else if (code == "R_A") {
					do c.Code('A', IsLeafAction);
					while (!c.AtEnd());
				} else if (code == "R_R") {
					c.Id();
				} else if (code == "R_E") {
					c.Id();
					c.Num();
				} else if (code == "R_K") {
					c.Tid();
				} else if (code == "R_C") {
					c.Label('C');
				}
			}

			void ActionSlots(Cursor& c, const std::string& code) {
				if (code == "A_MAP") {
					do {
						c.Id();
						c.Label('A');
					} while (!c.AtEnd());
					return;
				}
				if (code == "A_FN") {
					std::string target = c.Code('A', [](const std::string& a) { return IsLeafAction(a) && a != "A_N" && a != "A_I"; });
					const auto n = static_cast<std::size_t>(Device::ValueCount(Device::FindAction(target)->layout));
					c.Labels("F", n, n);
					return;
				}
				// Leaf action: values by layout, checked by packing (spec 02 §2).
				const auto layout = Device::FindAction(code)->layout;
				const Token* first = c.Peek();
				std::optional<std::uint64_t> packed;
				if (Device::IsIntegerLayout(layout)) {
					std::vector<std::int64_t> values;
					for (int k = 0; k < Device::ValueCount(layout); ++k) values.push_back(c.Int());
					packed = Device::PackArgument(layout, values);
				} else {
					std::vector<double> values;
					for (int k = 0; k < Device::ValueCount(layout); ++k) values.push_back(c.Num());
					packed = Device::PackArgument(layout, values);
				}
				if (!packed) Fail(ErrorClass::Slot, first->offset, "", code + ": value outside the layout's range");
			}

			void FunctionSlots(Cursor& c, const std::string& code) {
				if (code == "F_K") {
					c.Num();
				} else if (code == "F_L") {
					std::int64_t n = c.Count();
					for (std::int64_t k = 0; k < n; ++k) c.Num();
					c.Num();
					for (std::int64_t k = 0; k < n; ++k) c.Id();
				} else if (code == "F_X") {
					c.Id();
					std::int64_t n = c.Count();
					for (std::int64_t k = 0; k < n; ++k) c.Num();
				} else if (code == "F_S" || code == "F_P") {
					c.Labels("F", 1);
				} else if (code == "F_C") {
					c.Nums(2);
					c.Label('F');
				}
			}

			void ViewSlots(Cursor& c, const std::string& code) {
				c.Num();  // stamp
				static const std::map<std::string, int> plain = {
					{"V_GEO", 3}, {"V_POS3", 3}, {"V_VEL3", 3}, {"V_ATT", 3}, {"V_RATE", 3}, {"V_ACC", 3},
					{"V_BARO", 1}, {"V_MAG", 1}, {"V_RNG", 1}, {"V_BAT", 3}, {"V_P2", 3}};
				auto it = plain.find(code);
				if (it != plain.end()) return c.Nums(it->second);
				if (code == "V_PEER") {
					c.Int();
					c.Nums(7);
					c.Tid();
				} else if (code == "V_REL3") {
					c.Int();
					c.Nums(3);
				} else if (code == "V_DET") {
					c.Id();
					c.Nums(4);
				}
			}

			void DeviceSlots(Cursor& c, const std::string& code) {
				c.Id();
				c.Id();
				c.Params();
				if (code == "D_H" || code == "D_J") {
					c.Labels("D", 1);
					return;
				}
				c.Labels("P");
				c.Labels("K");
				if (code == "D_X") c.Labels("D");
			}

			const std::set<std::string> kPortTypes = {"GPIO", "PWM", "ADC", "UART", "I2C", "SPI",
			                                          "CAN",  "UDP", "TCP", "MAVLINK", "SIM"};

			void PortSlots(Cursor& c, const std::string& code) {
				if (code == "P_A") {
					c.Id();  // node
					c.Id();  // requirement
				}
				c.IdIn(kPortTypes);
				c.IdOrStr();
				if (code == "P_R") c.Bool();
				c.Params();
			}

			void CapabilitySlots(Cursor& c, const std::string& code) {
				if (code == "K_A") c.Code('A', IsLeafAction);
				else if (code == "K_V") c.Code('V', IsKnown);
				else if (code == "K_M") c.IdIn({"broadcast", "p2p"});
				else if (code == "K_Q") {
					c.Id();
					c.Num();
				}
				std::int64_t n = c.Count();
				for (std::int64_t k = 0; k < n; ++k) {
					c.Id();
					c.Num();
				}
			}

			void HeaderSlots(Cursor& c, const std::string&) {
				c.Id();
				c.Nums(11);
				std::int64_t n = c.Count();
				for (std::int64_t k = 0; k < n; ++k) {
					c.Int();
					c.Nums(3);
				}
			}

			void TimelineSlots(Cursor& c, const std::string&) {
				c.Num();
				c.Labels("T", 1);
			}

			void JournalSlots(Cursor& c, const std::string& code) {
				c.Num();
				if (code == "J_H") {
					c.Id();
					c.Label('H');
				} else if (code == "J_T") {
					c.IdIn(kPoolStates);
					c.Int();
					c.Label('T');
				} else if (code == "J_K") {
					std::int64_t n = c.Count();
					for (std::int64_t k = 0; k < n; ++k) c.Tid();
				} else if (code == "J_C") {
					c.Tid();
					c.Id();
					c.Num();
				} else if (code == "J_E") {
					c.IdIn({"start", "swap_land", "restart", "end"});
				}
			}

			void BehaviourSlots(Cursor& c, const std::string&) {
				c.Id();
				c.Code('C', IsKnown);
				c.Id();
				c.Num();
				c.Label('T');
			}

			void MessageSlots(Cursor& c, const std::string& code) {
				c.Num();   // ver
				c.Id();    // mission
				c.Id();    // sender
				c.Id();    // recipient
				c.Int();   // seq
				c.Num();   // stamp
				if (code == "M_MISSION") c.Label('H');
				else if (code == "M_TASK") c.Labels("T", 1);
				else if (code == "M_CLAIM") {
					c.Tid();
					c.Nums(2);
				} else if (code == "M_RELEASE" || code == "M_DONE") c.Tid();
				else if (code == "M_FAIL") {
					c.Tid();
					c.Id();
				} else if (code == "M_DUMP") {
					c.Tid();
					c.Id();
					c.Bool();
					c.Int();
				} else if (code == "M_STATE") c.Label('V');
				else if (code == "M_PROFILE") {
					c.Id();
					std::int64_t n = c.Count();
					for (std::int64_t k = 0; k < n; ++k) c.Id();
					c.Labels("K");
				} else if (code == "M_BID") {
					std::int64_t n = c.Count();
					for (std::int64_t k = 0; k < n; ++k) {
						c.Tid();
						c.Num();
						c.Id();
						c.Num();
					}
				} else if (code == "M_INFOREQ") {
					std::int64_t n = c.Count();
					for (std::int64_t k = 0; k < n; ++k) c.Id();
				} else if (code == "M_INFO") c.Labels("HTKV");
				else if (code == "M_PLAN") {
					c.Int();  // revision
					std::int64_t n = c.Count();
					for (std::int64_t k = 0; k < n; ++k) c.Tid();
				} else if (code == "M_CMD") {
					c.IdIn({"abort", "recall", "pause", "resume", "cancel"});
					c.Tid();
				}
			}

			const Schema& GetSchema() {
				static const Schema schema = [] {
					Schema s;
					auto add = [&s](std::initializer_list<const char*> codes, Slots slots) {
						for (const char* code : codes) s.codes[code] = slots;
					};
					add({"T_A", "T_P", "T_B", "T_S", "T_L", "T_O"}, TaskSlots);
					add({"C_N", "C_F", "C_?", "C_m", "C_T", "C_W", "C_L", "C_V", "C_S", "C_P", "C_P3", "C_G", "C_H"}, ConditionSlots);
					add({"R_S", "R_F", "R_A", "R_R", "R_E", "R_K", "R_C"}, RequirementSlots);
					add({"A_MAP", "A_FN"}, ActionSlots);
					for (const auto& info : Device::ActionRegistry()) s.codes[info.code] = ActionSlots;
					add({"F_K", "F_L", "F_X", "F_S", "F_P", "F_C"}, FunctionSlots);
					add({"V_GEO", "V_POS3", "V_VEL3", "V_ATT", "V_RATE", "V_ACC", "V_BARO", "V_MAG", "V_RNG", "V_BAT",
					     "V_PEER", "V_REL3", "V_DET", "V_P2"}, ViewSlots);
					add({"D_H", "D_J", "D_X", "D_S", "D_A", "D_C", "D_M"}, DeviceSlots);
					add({"P_R", "P_A"}, PortSlots);
					add({"K_A", "K_V", "K_M", "K_Q"}, CapabilitySlots);
					add({"H_M"}, HeaderSlots);
					add({"L_D"}, TimelineSlots);
					add({"J_H", "J_T", "J_K", "J_C", "J_E"}, JournalSlots);
					add({"B_E"}, BehaviourSlots);
					add({"M_MISSION", "M_TASK", "M_CLAIM", "M_RELEASE", "M_DONE", "M_FAIL", "M_DUMP", "M_STATE",
					     "M_PROFILE", "M_BID", "M_INFOREQ", "M_INFO", "M_CMD", "M_PLAN"}, MessageSlots);
					s.reserved = {"T_D", "T_G", "T_U", "C_Q", "F_E",
					              // legacy 2021 action codes (spec 02 §4.5)
					              "A_l", "A_p", "A_t", "A_f", "A_b", "A_r", "A_y", "A_c", "A_d", "A_D", "A_M", "A_R",
					              "A_K", "A_T", "A_C", "A_V"};
					return s;
				}();
				return schema;
			}

			bool IsSchemaCode(const std::string& code) {
				const Schema& s = GetSchema();
				return s.codes.count(code) != 0 && s.reserved.count(code) == 0;
			}

			const std::string kKindsWithCode = "TCRAFVDPKMHLJB";

			// Checks that need the children's codes.
			void PostCheck(const Record& rec, const std::string& header) {
				auto fail_child = [&](const Record& child, const std::string& msg) {
					Fail(ErrorClass::Slot, child.code_offset, header, msg);
				};
				if (rec.code == "C_V") {
					const Record& v = rec.children.at(0);
					if (v.code != "V_DET" && v.code != "V_PEER") fail_child(v, "C_V: only V_DET and V_PEER can be matched in version 0.1");
				} else if (rec.code == "M_STATE") {
					if (rec.children.at(0).code != "V_PEER") fail_child(rec.children[0], "M_STATE carries a V_PEER view");
				} else if (rec.code == "A_MAP") {
					std::set<std::pair<std::string, std::string>> entries;
					for (std::size_t k = 0; k + 1 < rec.fields.size(); k += 2) {
						const Record& a = rec.children.at(rec.fields[k + 1].ref);
						if (a.code == "A_MAP" || a.code == "A_N" || a.code == "A_W" || a.code == "A_I")
							fail_child(a, "A_MAP: " + a.code + " is not allowed inside a map");
						const std::string& produced = a.code == "A_FN" ? a.fields.at(0).s : a.code;
						if (!entries.insert({rec.fields[k].s, produced}).second)
							fail_child(a, "A_MAP: two entries address the same target with the same code");
					}
				} else if (rec.code == "P_A") {
					const Field& address = rec.fields.at(3);
					if (address.type == FieldType::Id && address.s == "any")
						Fail(ErrorClass::Slot, address.offset, header, "P_A: a port map entry needs a fixed address, not any");
				} else if (rec.code == "T_A") {
					std::function<void(const Record&)> no_fn = [&](const Record& r) {
						if (r.code == "A_FN") fail_child(r, "T_A: actions must not include A_FN (use T_P)");
						for (const auto& child : r.children)
							if (child.kind == 'A') no_fn(child);
					};
					for (const auto& child : rec.children)
						if (child.kind == 'A') no_fn(child);
				}
			}

			class TreeParser {
			public:
				explicit TreeParser(std::vector<RawRecord> raw, std::size_t text_size) : raw_(std::move(raw)), text_size_(text_size) {}

				Document Run() {
					Document doc;
					while (pos_ < raw_.size()) {
						const RawRecord& r = raw_[pos_];
						if (r.header.size() != 1) Fail(ErrorClass::Reference, r.offset, r.header, "sub-record without a parent");
						doc.records.push_back(ParseRecord(r.header[0]));
					}
					return doc;
				}

			private:
				Record ParseRecord(char kind) {
					const RawRecord& raw = raw_[pos_++];
					Record rec;
					rec.kind = kind;
					rec.offset = raw.offset;
					if (kind == '@' || kind == '?') {
						ParseLiteral(raw, rec);
						return rec;
					}
					if (kKindsWithCode.find(kind) == std::string::npos)
						Fail(ErrorClass::Code, raw.offset, raw.header, std::string("unknown kind ") + kind);
					if (raw.tokens.empty()) Fail(ErrorClass::Slot, raw.end_offset, raw.header, "missing code");
					const Token& code = raw.tokens[0];
					if (code.type != Tok::Code) Fail(ErrorClass::Slot, code.offset, raw.header, "a record body starts with a code");
					rec.code = std::string(code.text);
					rec.code_offset = code.offset;
					const Schema& schema = GetSchema();
					if (schema.reserved.count(rec.code)) Fail(ErrorClass::Code, code.offset, raw.header, "reserved code " + rec.code);
					auto it = schema.codes.find(rec.code);
					if (it == schema.codes.end()) Fail(ErrorClass::Code, code.offset, raw.header, "unknown code " + rec.code);
					if (code.kind != kind) Fail(ErrorClass::Code, code.offset, raw.header, "code " + rec.code + " does not match the header");

					Cursor cursor(raw, rec);
					try {
						it->second(cursor, rec.code);
						cursor.End();
					} catch (Failure& f) {
						f.error.header = raw.header;
						throw;
					}

					for (const std::string& label : cursor.refs) {
						if (pos_ >= raw_.size())
							Fail(ErrorClass::Reference, text_size_, raw.header, "sub-record " + label + " is missing");
						const RawRecord& next = raw_[pos_];
						if (next.header != label)
							Fail(ErrorClass::Reference, next.offset, next.header, "expected sub-record " + label + ", found " + next.header);
						rec.children.push_back(ParseRecord(label[0]));
					}
					PostCheck(rec, raw.header);
					return rec;
				}

				void ParseLiteral(const RawRecord& raw, Record& rec) {
					const auto& t = raw.tokens;
					auto slot_error = [&](std::size_t k, const char* msg) {
						Fail(ErrorClass::Slot, k < t.size() ? t[k].offset : raw.end_offset, raw.header, msg);
					};
					if (rec.kind == '?') {
						if (t.size() < 1 || t[0].type != Tok::Ident) slot_error(0, "predicate: expected a name");
						if (t.size() < 2 || t[1].type != Tok::Bool) slot_error(1, "predicate: expected T or F");
						if (t.size() > 2) slot_error(2, "predicate: too many slots");
						rec.fields.push_back(Field::MakeText(FieldType::Id, std::string(t[0].text)));
						rec.fields.push_back(Field::MakeBool(t[1].text == "T"));
						return;
					}
					if (t.size() < 1 || t[0].type != Tok::Ident || t[0].text != "MRS") slot_error(0, "version: expected MRS");
					if (t.size() < 2 || (t[1].type != Tok::Num && t[1].type != Tok::Int)) slot_error(1, "version: expected a number");
					if (t.size() > 2) slot_error(2, "version: too many slots");
					double v;
					std::from_chars(t[1].text.data(), t[1].text.data() + t[1].text.size(), v);
					if (v < 0 || v >= 1) Fail(ErrorClass::Version, t[1].offset, raw.header, "unsupported major version");
					rec.fields.push_back(Field::MakeText(FieldType::Id, "MRS"));
					rec.fields.push_back(Field::MakeNum(v));
				}

				std::vector<RawRecord> raw_;
				std::size_t text_size_;
				std::size_t pos_ = 0;
			};
		}

		ParseResult Parse(std::string_view text) {
			ParseResult result;
			try {
				result.document = TreeParser(Lex(text), text.size()).Run();
			} catch (const Failure& f) {
				result.error = f.error;
			}
			return result;
		}
	}
}
