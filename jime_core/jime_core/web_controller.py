class WebControllerNode(Node):
    def __init__(self):
        super().__init__('web_controller_node')
        # This sends the "Button Clicked" signal to the Brain
        self.ui_pub = self.create_publisher(String, '/ui_interaction', 10)

# Inside the Flask/SocketIO handler:
@socketio.on('ui_button_click')
def handle_ui_click(json):
    msg = String()
    msg.data = json['action'] # e.g., "CONFIRM_START"
    ros_node.ui_pub.publish(msg)
