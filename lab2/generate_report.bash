#!/bin/bash
jupyter nbconvert lab2-report.ipynb --template-file=pdf-format.tpl --no-prompt --no-input --to webpdf
