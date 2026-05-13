import os
import sys

project = "GNSS/INS Navigation — C++ Migration"
copyright = "2012, Paul D. Groves (original MATLAB). C++ port documentation."
author = "Paul D. Groves / C++ Port Contributors"
release = "2.0-cpp"
version = "2.0"

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "sphinx.ext.intersphinx",
    "sphinx.ext.todo",
]

myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "tasklist",
]

templates_path = ["_templates"]
exclude_patterns = []
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
master_doc = "index"

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_css_files = ["custom.css"]

html_theme_options = {
    "logo_only": False,
    "prev_next_buttons_location": "bottom",
    "style_external_links": True,
    "collapse_navigation": False,
    "sticky_navigation": True,
    "navigation_depth": 4,
    "includehidden": True,
    "titles_only": False,
}

html_context = {
    "display_github": True,
}

todo_include_todos = True
pygments_style = "monokai"
highlight_language = "cpp"
