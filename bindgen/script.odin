package script

import "core:slice"
import "core:log"
import "core:strings"
import "core:text/regex"
import "core:odin/ast"
import "core:odin/parser"
import os "core:os/os2"

BINDGEN_PATH :: "odin-c-bindgen/bindgen.exe"
GENERATED_PATH :: "bindgen/generated/joltc.odin"
HEADER_PATH :: "joltc/include/joltc.h"
FINAL_PATH :: "jolt/joltc.odin"

main :: proc() {
    context.logger = log.create_console_logger()
	run({BINDGEN_PATH, "bindgen"})

	// fix what bindgen couldn't handle. we should probably preprocess the .h file instead but oh well, Karl will make all this obsolete anyway :D
	bindings := read_text_file(GENERATED_PATH)

	p := parser.default_parser()
	ast_file := ast.File {
		src = bindings,
		fullpath = GENERATED_PATH,
	}
	parse_ok := parser.parse_file(&p, &ast_file); assert(parse_ok)

	bindings_builder: strings.Builder
	strings.builder_init(&bindings_builder)

	// would love to use the parser instead of regex for everything, but there's no printer in the odin core collection
	enum_start_regex, regex_err := regex.create("(\\w+) :: enum i32 \\{"); assert(regex_err == nil)
	enum_variant_regex, regex_err1 := regex.create("^(\\s*)(JPH|_JPH)_(\\w+)_(\\w+)(.*)$"); assert(regex_err1 == nil)

	binding_lines := bindings
	in_enum: Maybe(string)
	for l in strings.split_lines_iterator(&binding_lines) {
		// (~0) to max(u32)
		line, _ := strings.replace_all(l, "(~0)", "max(u32)", context.temp_allocator)

		enum_start_capture, enum_start_matched := regex.match_and_allocate_capture(enum_start_regex, line)
		if enum_start_matched do in_enum = enum_start_capture.groups[1]

		enum_variant_capture, enum_variant_matched := regex.match_and_allocate_capture(enum_variant_regex, line)
		if enum_variant_matched {
			lead := enum_variant_capture.groups[1]
			prefix := enum_variant_capture.groups[2]
			enum_name := enum_variant_capture.groups[3]
			enum_variant_name := enum_variant_capture.groups[4]
			trail := enum_variant_capture.groups[5]

			// not sure what's about this double prefix there
			if enum_name == "JPH_OverrideMassProperties" do enum_name = "OverrideMassProperties"
			ensure(enum_name == in_enum)

			// we don't need those, in Odin you can just do `len(Enum)`
			if prefix == "_JPH" && (enum_variant_name == "Count" || enum_variant_name == "Force32") do continue

			line = strings.concatenate({ lead, enum_variant_name, trail })
		}

		strings.write_string(&bindings_builder, line)
		strings.write_byte(&bindings_builder, '\n')
	}

	// empty opaque structures
	opaque_typedef_regex, regex_err2 := regex.create("^typedef struct (JPH_(\\w+))\\s+[^{]+$"); assert(regex_err2 == nil)
	forward_structs: map[string]struct{}
	header_lines := read_text_file(HEADER_PATH)
	for line in strings.split_lines_iterator(&header_lines) {
		capture, matched := regex.match_and_allocate_capture(opaque_typedef_regex, line)
		if !matched do continue
		forward_structs[capture.groups[2]] = {}
	}

	// filter structs that were forward-declared but are not opaque, so they are generated in the bindings by bindgen
	for decl in ast_file.decls {
		v := decl.derived.(^ast.Value_Decl) or_continue
		_ = v.values[0].derived.(^ast.Struct_Type) or_continue
		n := v.names[0].derived_expr.(^ast.Ident).name
		delete_key(&forward_structs, n)
	}

	// write empty opaque structs to the bindings file
	strings.write_byte(&bindings_builder, '\n')
	struct_names, err := slice.map_keys(forward_structs); assert(err == nil)
	slice.sort(struct_names) // for stable output
	for name in struct_names {
		strings.write_string(&bindings_builder, name)
		strings.write_string(&bindings_builder, " :: struct {}\n")
	}

	bindings = strings.to_string(bindings_builder)
	write_err := os.write_entire_file(FINAL_PATH, transmute([]byte)bindings); assert(write_err == nil)
}

read_text_file :: proc(path: string) -> string {
	data, err := os.read_entire_file_from_path(path, context.temp_allocator)
	assert(err == nil)
	return string(data)
}

run :: proc(cmd: []string) {
	log.infof("Running {}", cmd)
	code, err := exec(cmd)
	if err != nil {
		log.errorf("Error executing process: {}", err)
		os.exit(1)
	}
	if code != 0 {
		log.errorf("Process exited with non-zero code {}", code)
		os.exit(1)
	}
}

exec :: proc(cmd: []string) -> (code: int, error: os.Error) {
	process := os.process_start(
		{
			command = cmd,
			stdin = os.stdin,
			stdout = os.stdout,
			stderr = os.stderr,
			env = os.environ(context.temp_allocator) or_return,
			working_dir = os.get_working_directory(context.temp_allocator) or_return,
		},
	) or_return
	state := os.process_wait(process) or_return
	os.process_close(process) or_return
	return state.exit_code, nil
}
