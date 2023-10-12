{%- extends 'webpdf/index.pdf.j2' %}

{% block html_head %}
<style>
img {
  display: table-cell;
  margin-left: auto;
  margin-right: auto;
  max-height: 180px;
  max-width: 80%;
}
h1 {
  text-align: center;
}
</style>
{{ super() }}
{% endblock html_head %}
