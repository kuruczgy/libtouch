project('libtouch', 'c', default_options : [ 'c_std=c11' ])
cc = meson.get_compiler('c')
m = cc.find_library('m', required : false)

libtouch = library('libtouch', 'libtouch.c', dependencies : m)
libtouch_dep = declare_dependency(include_directories: '.',
  link_with : libtouch)
