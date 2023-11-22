#!/bin/bash
jupyter nbconvert lab3-report.ipynb --template-file=pdf-format.tpl --no-prompt --no-input --to webpdf
