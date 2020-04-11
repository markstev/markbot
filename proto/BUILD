package(default_visibility = ["//visibility:public"])

# load("@com_github_silvergasp_nanopb//rules:nanopb_proto_library.bzl",
#      "nanopb_proto_library")

# nanopb_proto_library(
#   name = "motor_command",
#   transitive = True,
#   deps = ["@com_google_protobuf//:descriptor_proto"],
# )

genrule(
		name = "motor_command_generator",
		srcs = ["motor_command.proto"],
		outs = ["_motor_command.pb.h",
						"_motor_command.pb.c"],
		cmd = "python3 /Users/mark/nanopb/generator/nanopb_generator.py $(location motor_command.proto) \
				&& echo yolo && which sed && sed 's/#include <pb.h>/#include \"pb.h\"/' proto/motor_command.pb.h \
        > $(location _motor_command.pb.h) \
				&& cp proto/motor_command.pb.c $(location _motor_command.pb.c)",
		#tools = ["/Users/mark/nanopb/generator/nanopb_generator.py"],
)

genrule(
    name = "motor_command_generator_h",
    srcs = [":motor_command_generator"],
    outs = ["motor_command.pb.h"],
    cmd = "cp $$(echo $(locations :motor_command_generator) | sed 's/[^ \t]*\.c[^ \t]*//') $@"
)

genrule(
    name = "motor_command_generator_c",
    srcs = [":motor_command_generator"],
    outs = ["motor_command.pb.c"],
    cmd = "cp $$(echo $(locations :motor_command_generator) | sed 's/[^ \t]*\.h$$//') $@"
)

cc_library(
		name = "motor_command_proto",
		srcs = [":motor_command_generator_c"],
		hdrs = [":motor_command_generator_h"],
		deps = ["@com_github_nanopb_nanopb//:nanopb"],
		)

cc_library(
		name = "motor_command_proto2",
		hdrs = ["motor_command.pb.h"],
		srcs = ["motor_command.pb.c"],
		deps = ["@com_github_nanopb_nanopb//:nanopb"],
)
