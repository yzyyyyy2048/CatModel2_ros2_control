import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QTableWidget, QTableWidgetItem
import yaml
from yaml.constructor import ConstructorError  
from yaml.loader import SafeLoader  
from std_msgs.msg import String

# 自定义构造函数，用于处理特殊标签  
def construct_custom_float(loader, node):  
    value = loader.construct_scalar(node)  
    return float(value.replace('!float ', ''))  
  
def construct_custom_int(loader, node):  
    value = loader.construct_scalar(node)  
    return int(value.replace('!int ', ''))  
  
def construct_custom_bool(loader, node):  
    value = loader.construct_scalar(node)  
    return value.lower() == 'true' 

# 自定义加载器，使用自定义构造函数  
class CustomLoader(SafeLoader):  
    def __init__(self, stream):  
        super(CustomLoader, self).__init__(stream)  
        self.add_constructor('!float', construct_custom_float)  
        self.add_constructor('!int', construct_custom_int)  
        self.add_constructor('!bool', construct_custom_bool)  

class MyPlugin(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle('YAML Viewer')
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # Create a label for the YAML file path
        self.yaml_file_path_label = QLabel('YAML file path:')
        self.layout.addWidget(self.yaml_file_path_label)

        # Create a line edit for the YAML file path
        self.yaml_file_path_line_edit = QLineEdit()
        self.layout.addWidget(self.yaml_file_path_line_edit)

        # Create a button to load the YAML file
        self.load_yaml_file_button = QPushButton('Load YAML file')
        self.load_yaml_file_button.clicked.connect(self.load_yaml_file)
        self.layout.addWidget(self.load_yaml_file_button)

        # Create a label for the data
        self.data_label = QLabel('Data:')
        self.layout.addWidget(self.data_label)

        # Create a table for the data
        self.data_table = QTableWidget()
        self.data_table.setColumnCount(3)
        self.data_table.setHorizontalHeaderLabels(['Name', 'Type', 'Value'])
        self.layout.addWidget(self.data_table)

        # Create a button to send the data to ROS2
        self.send_data_button = QPushButton('Send data to ROS2')
        self.send_data_button.clicked.connect(self.send_data)
        self.layout.addWidget(self.send_data_button)

        # Store the original data from the YAML file
        self.original_data = {}
        self.original_data_type = {}
        # Initialize ROS2 node and publisher
        self.node = rclpy.create_node('yaml_viewer')
        self.publisher = None

    def load_yaml_file(self):
        # Get the YAML file path from the line edit
        yaml_file_path = self.yaml_file_path_line_edit.text()

        # Load the YAML file
        with open(yaml_file_path, 'r') as f:
            self.original_data = yaml.load(f, Loader=CustomLoader)
            self.original_data_type = self.original_data.copy()
        # Clear the data table
        self.data_table.clearContents()
        self.data_table.setRowCount(0)

        # Add the data to the data table
        for name, value in self.original_data.items():
            row = self.data_table.rowCount()
            self.data_table.insertRow(row)
            self.data_table.setItem(row, 0, QTableWidgetItem(name))
            self.data_table.setItem(row, 1, QTableWidgetItem(type(value).__name__))
            self.data_table.setItem(row, 2, QTableWidgetItem(str(value)))

    def send_data(self):
        # Get the modified data from the data table
        modified_data = {}
        for row in range(self.data_table.rowCount()):
            name = self.data_table.item(row, 0).text()
            value = self.data_table.item(row, 2).text()
            original_value = self.original_data.get(name)
            if original_value is not None and str(original_value) != value:
                if isinstance(self.original_data_type[name], bool):
                    modified_data[name] = bool(value == "True")
                elif isinstance(self.original_data_type[name], int):
                    modified_data[name] = int(value)
                elif isinstance(self.original_data_type[name], float):
                    modified_data[name] = float(value)
                else:
                    modified_data[name] = value

        # Send the modified data to ROS2
        if modified_data:
            if self.publisher is None:
                self.publisher = self.node.create_publisher(String, 'parameter_tuning', 10)

            for name, value in modified_data.items():
                data_type = type(self.original_data_type[name]).__name__
                modified_string = f"{name}@{data_type}@{value}"

                msg = String()
                msg.data = modified_string
                self.publisher.publish(msg)
                print(f'Sending modified data to ROS2: {modified_string}')

                # Update the original_data with the modified data
                self.original_data[name] = value

                for row in range(self.data_table.rowCount()):
                    modify_name = self.data_table.item(row, 0).text()
                    if modify_name == name:
                        value = self.original_data[name]
                        self.data_table.setItem(row, 2, QTableWidgetItem(str(value)))

    def closeEvent(self, event):
        # Update the data table with the modified data before closing the GUI
        for row in range(self.data_table.rowCount()):
            name = self.data_table.item(row, 0).text()
            if name in self.original_data:
                value = self.original_data[name]
                self.data_table.setItem(row, 2, QTableWidgetItem(str(value)))

        super().closeEvent(event)


def main():
    # Initialize ROS2 node
    rclpy.init()

    # Create the application
    app = QApplication([])

    # Create an instance of the MyPlugin class
    my_plugin = MyPlugin()
    my_plugin.show()

    # Load a sample YAML file
    my_plugin.yaml_file_path_line_edit.setText('src/CatModel2_v2_description/config/ocs2/parameter.yaml')
    
    my_plugin.load_yaml_file()

    # Run the application
    app.exec_()

    # Clean up ROS2 resources
    my_plugin.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()