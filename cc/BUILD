package(default_visibility = ["//visibility:public"])

cc_library(name = "module_dispatcher",
           srcs = ["module_dispatcher.cc"],
           hdrs = ["module_dispatcher.h"],
)

cc_library(name = "serial_module",
           srcs = ["serial_module.cc"],
           hdrs = ["serial_module.h"],
           deps = [
               ":module_dispatcher",
               "@markstev_tensixty//cc:commlink",
               "@markstev_tensixty//cc:interfaces"
           ]
)
