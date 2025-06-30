# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ByteRider'
copyright = '2025, Alexander B.'
author = 'Alexander B.'
release = '06-2025'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx_simplepdf",
]

templates_path = ['_templates']
include_patterns = ['*.rst', '*.md']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']

# -- Options for simplepdf output --------------------------------------------
#simplepdf_css = '_static/simplepdf_alex.css'
simplepdf_coverpage = True
simplepdf_toc_depth = 3
simplepdf_title = 'ESP-IDF ESPNOW RC Car'
simplepdf_author = 'Alexander B.'
simplepdf_file_name = 'esp-idf_espnow_rc-car.pdf'
simplepdf_vars = {
#    'cover-overlay': '#047e2c',
#    'cover-bg': "#034b1b",
#    'primary-opaque': 'rgba(26, 150, 26, 0.8)',
    'primary': "#1A961A",
    'secondary': "#379683",
    'cover-bg': 'url(cover-bg.jpg) no-repeat center',
#   'cover-bg': 'url(ESP32C3_Breadboard-Adapter.jpg) no-repeat center',
    'cover': "#DADAE4",
    'links': "#790000",
    'bottom-center-content': '"Getting RC onn the road"',
    'bottom-right-content': '"Alexander B."',
}
