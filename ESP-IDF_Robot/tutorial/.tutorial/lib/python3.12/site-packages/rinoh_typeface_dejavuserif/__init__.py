from os import path

from rinoh.font import Typeface
from rinoh.font.style import REGULAR, BOLD, ITALIC, FontWidth
from rinoh.font.opentype import OpenTypeFont


__all__ = ['typeface']


try:
    SEMI_CONDENSED = FontWidth.SEMI_CONDENSED
except AttributeError:  # rinohtype <= 0.4.2
    SEMI_CONDENSED = FontWidth.CONDENSED


def otf(style, **kwargs):
    filename = 'DejaVuSerif{}.ttf'.format(style)
    return OpenTypeFont(path.join(path.dirname(__file__), filename), **kwargs)


typeface = Typeface('DejaVu Serif',
                    otf('', weight=REGULAR),
                    otf('-Italic', weight=REGULAR, slant=ITALIC),
                    otf('-Bold', weight=BOLD),
                    otf('-BoldItalic', weight=BOLD, slant=ITALIC),
                    otf('Condensed', weight=REGULAR, width=SEMI_CONDENSED),
                    otf('Condensed-Italic', weight=REGULAR, slant=ITALIC,
                        width=SEMI_CONDENSED),
                    otf('Condensed-Bold', weight=BOLD, width=SEMI_CONDENSED),
                    otf('Condensed-BoldItalic', weight=BOLD, slant=ITALIC,
                        width=SEMI_CONDENSED))
