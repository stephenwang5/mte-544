{%- extends 'webpdf/index.pdf.j2' %}

{% block html_head %}
<style>
img {
  display: table-cell;
  margin-left: auto;
  margin-right: auto;
  max-height: 280px;
  max-width: 100%;
  min-width: 50%;
}
h1 {
  text-align: center;
}
.jp-CodeCell {
  --jp-cell-padding: 0px;
}
</style>
{{ super() }}
{% endblock html_head %}
