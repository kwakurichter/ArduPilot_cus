# encoding: utf-8

# flake8: noqa

"""
Adds support for building the Bitcraze libdw1000 driver as part of a Waf build.

Only the portable driver core under src/ is compiled; the test/ and vendor/
(Unity, CMock) trees in the submodule are ignored.

This lives here rather than as a wscript inside modules/libdw1000 so that it is
tracked by this repository. The submodule points at upstream Bitcraze, which
carries no waf build, so anything placed inside it is untracked and lost on a
fresh clone. This mirrors how littlefs is built, in littlefs.py.
"""

from waflib.Configure import conf

def configure(cfg):
    cfg.env.append_value('GIT_SUBMODULES', 'libdw1000')
    cfg.env.prepend_value('INCLUDES', [
        cfg.srcnode.abspath() + '/modules/libdw1000/inc',
    ])


@conf
def libdw1000(bld, **kw):
    kw.update(
        name='libdw1000',
        source=['modules/libdw1000/src/libdw1000.c',
                'modules/libdw1000/src/libdw1000Spi.c'],
        target='libdw1000',
        includes=['modules/libdw1000/inc'],
        export_includes=['modules/libdw1000/inc'],
    )
    return bld.stlib(**kw)
