# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "ECE 5160: Fast Robots"
copyright = "2025, Aidan McNay"
author = "Aidan McNay"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx_rtd_theme",
    "sphinxcontrib.youtube",
]

templates_path = ["_templates"]
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_context = {
    "display_github": True,  # Integrate GitHub
    "github_user": "Aidan-McNay",  # Username/Organization
    "github_repo": "fast-robots-docs",  # Repo name
    "github_version": "main",  # Version
    "conf_py_path": "/docs/",  # Path in the checkout to the docs root
}

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_favicon = "_static/img/favicon.ico"
html_css_files = ["css/theme_overrides.css"]

# html_theme_options = {
#     "navigation_depth": 3,
# }
