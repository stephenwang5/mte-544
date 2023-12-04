#!/bin/bash
jupyter nbconvert lab4-report.ipynb --template-file=pdf-format.tpl --no-prompt --no-input --to webpdf
