PANDOC = pandoc
RM = rm -f
DOCX_FILE = scan_operation.docx
TARGET_FILES = ${DOCX_FILE}

.phony: clean docx

all: docx

pdf: ${PDF_FILE}

docx: ${DOCX_FILE}

%.docx:%.md
	${PANDOC} -o $@ -s $<

clean:
	${RM} ${TARGET_FILES}