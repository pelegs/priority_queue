validation:
	rm -rf *.pytxcode pythontex-files-validation
	pdflatex validation
	pythontex validation.pytxcode
	pdflatex validation
