validation:
	rm -rf *.pytxcode pythontex-files-validation temp.txt
	pdflatex validation
	pythontex validation.pytxcode
	pdflatex validation
