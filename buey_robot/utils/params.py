"""Declara params ROS2 (sin default, fail-fast) desde una tabla y los asigna al nodo."""

from rclpy.parameter import Parameter

_T = {str: Parameter.Type.STRING, int: Parameter.Type.INTEGER,
      float: Parameter.Type.DOUBLE, bool: Parameter.Type.BOOL,
      list: Parameter.Type.DOUBLE_ARRAY}


def load_params(node, spec):
    """spec: {attr: type} o {attr: (param_name, type)} si el nombre difiere (dotted).
    Deja cada valor como atributo de node (node.<attr>)."""
    for attr, v in spec.items():
        param, typ = v if isinstance(v, tuple) else (attr, v)
        node.declare_parameter(param, _T[typ])
        setattr(node, attr, node.get_parameter(param).value)
