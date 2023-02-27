from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.button import Button


import rospy
from std_msgs.msg import String



START = "1"
STOP  = "2"
SEND  = "3"
PERS  = "4"
SAVE  = "5"

pub = rospy.Publisher('scan_command', String, queue_size=10)

class MainGrid(GridLayout):
    def __init__(self, **kwargs):
        super(MainGrid, self).__init__(**kwargs)

        self.cols = 1 
        self.names_grid = GridLayout() 
        self.names_grid.cols = 5 
        self.names_grid.add_widget(Label(text="[1] Start scanner"))
        self.names_grid.add_widget(Label(text="[2] Stop scanner"))
        self.names_grid.add_widget(Label(text="[3] Publish surface"))
        self.names_grid.add_widget(Label(text="[4] Publish persistent"))
        self.names_grid.add_widget(Label(text="[5] Save persistent"))
        self.add_widget(self.names_grid) 

        self.buttons_grid = GridLayout() 
        self.buttons_grid.cols = 5 
        self.btn1 = Button(text="Start", font_size=40)
        self.btn2 = Button(text="Stop", font_size=40)
        self.btn3 = Button(text="Surface", font_size=40)
        self.btn4 = Button(text="Persis", font_size=40)
        self.btn5 = Button(text="Save", font_size=40)

        self.buttons_grid.add_widget(self.btn1)
        self.buttons_grid.add_widget(self.btn2)
        self.buttons_grid.add_widget(self.btn3)
        self.buttons_grid.add_widget(self.btn4)
        self.buttons_grid.add_widget(self.btn5)

        self.add_widget(self.buttons_grid) 
        
        self.btn1.bind(on_press=self.btn1_pressed)
        self.btn2.bind(on_press=self.btn2_pressed)
        self.btn3.bind(on_press=self.btn3_pressed)
        self.btn4.bind(on_press=self.btn4_pressed)
        self.btn5.bind(on_press=self.btn5_pressed)


    def btn1_pressed(self, instance):
        command = START
        pub.publish(command)

    def btn2_pressed(self, instance):
        command = STOP
        pub.publish(command)

    def btn3_pressed(self, instance):
        command = SEND
        pub.publish(command)

    def btn4_pressed(self, instance):
        command = PERS
        pub.publish(command)

    def btn5_pressed(self, instance):
        command = SAVE
        pub.publish(command)
    


# START = 1
# STOP  = 2
# SEND  = 3
# PERS  = 4
# SAVE  = 5




class PanelApp(App):
    def build(self):
        return MainGrid()

if __name__ == "__main__":
    rospy.init_node('ros_gui', anonymous=True)
    rate = rospy.Rate(10) 
    PanelApp().run()
