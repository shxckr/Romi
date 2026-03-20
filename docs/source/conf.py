import os
import sys
sys.path.insert(0, os.path.abspath("../.."))

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ME 405 Term Project'
copyright = '2026, Fred Agregado, Samantha Kenney, Sara Chamness'
author = 'Fred Agregado, Samantha Kenney, Sara Chamness'
release = '3/20/2026'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.githubpages",
    "sphinxcontrib.video",
    "sphinx.ext.napoleon",
    "sphinx.ext.githubpages",
]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme' #alabaster
html_theme_options = {
    "titles_only": True,
    "collapse_navigation": False,
    "navigation_depth": 3,
}
html_static_path = ['_static']

autodoc_member_order = "bysource"
autodoc_default_options = {}
add_module_names = False
autoclass_content = "both"
autodoc_mock_imports = ["pyb", "utime", "micropython"]
