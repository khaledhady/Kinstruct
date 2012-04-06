@echo off
%3
cd %1
makeindex < %2.idx > %2.ind
bibtex %2
latex --src-specials %2.tex
dvips %2.dvi
