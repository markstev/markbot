load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# GoogleTest/GoogleMock framework. Used by most unit-tests.
http_archive(
     name = "google_googletest",
     urls = ["https://github.com/google/googletest/archive/b6cd405286ed8635ece71c72f118e659f4ade3fb.zip"],  # 2019-01-07
     strip_prefix = "googletest-b6cd405286ed8635ece71c72f118e659f4ade3fb",
     sha256 = "ff7a82736e158c077e76188232eac77913a15dac0b22508c390ab3f88e6d6d86",
)

git_repository(
		name = "markstev_tensixty",
		remote = "http://github.com/markstev/tensixty.git",
    commit = "fa8369382b6712422e9bae1c68c659c7c41fd32d",
)

git_repository(
    name = "com_github_nanopb_nanopb",
    remote = "https://github.com/nanopb/nanopb.git",
    commit = "58699f8a737823eabe001d4c1cf7ab2684ced861",
)

git_repository(
    name = "com_github_silvergasp_nanopb",
    remote = "https://github.com/silvergasp/nanopb",
    commit = "39f40c8acb034efe6e2e8be6785e2cc851d42e57",
)

git_repository(
    name = "com_google_protobuf",
    commit = "09745575a923640154bcf307fba8aedff47f240a",
    remote = "https://github.com/protocolbuffers/protobuf",
    shallow_since = "1558721209 -0700",
)
