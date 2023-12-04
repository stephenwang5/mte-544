{%- extends 'webpdf/index.pdf.j2' %}

{% block html_head %}
<style>
img {
  display: table-cell;
  margin-left: auto;
  margin-right: auto;
  max-height: 180px;
  max-width: 100%;
}
h1 {
  text-align: center;
}
.jp-CodeCell {
  --jp-cell-padding: 0px;
}
MathJax_CHTML {
  font-size: 80%;
}
body {
  --jp-content-font-size1: 9pt;
  --jp-code-font-size: 6pt;
  --jp-ui-font-size1: 8pt;
  --jp-content-font-family: serif;
}
</style>
{{ super() }}
{% endblock html_head %}
