package(default_visibility = ["//visibility:public"])


py_library(
    name='genpy',
    srcs=glob(['src/**/*.py']),
    imports=['src'],
    deps=[
    	'@genmsg_repo//:genmsg'
    ],
)


py_binary(
	name='genmsg_py',
	srcs=['scripts/genmsg_py.py'],
	deps=[
		':genpy'
	],
)