#!/usr/bin/env python
import rospy
import pygame
import gazebo_msgs.msg
import hector_uav_msgs.msg


total_points = 0
prev_count = 2
first_taken = False
end_l_one = (0, 0)
end_l_two = (0, 0)
end_l_three = (0,0)
pygame.init()
S = pygame.display.set_mode((700, 700))


def handle_model_states(msg):
	global prev_count,end_l_one,end_l_two,end_l_three
	try:
		n = len(msg.name)
		if n > prev_count:
			r_pix_x = int(350 + msg.pose[1].position.x * 35)
			r_pix_y = int(350 - msg.pose[1].position.y * 35)
			pygame.draw.circle(pygame.display.get_surface(), (255, 0, 0), (r_pix_x, r_pix_y), 3, 0)
			for i in range(1, n):
				if "uav" in msg.name[i]:
					if msg.name[i] == "uav1":
						end_l_one = (msg.pose[i].position.x,msg.pose[i].position.y)
					elif msg.name[i] == "uav2":
						end_l_two = (msg.pose[i].position.x,msg.pose[i].position.y)
					elif msg.name[i] == "uav3":
						end_l_three = (msg.pose[i].position.x,msg.pose[i].position.y)
					continue
				o_pix_x = int(350 + msg.pose[i].position.x * 35)
				o_pix_y = int(350 - msg.pose[i].position.y * 35)
				pygame.draw.circle(pygame.display.get_surface(), (220, 220, 220, 100), (o_pix_x, o_pix_y), 42, 0)
				
				
			prev_count = n
	except IndexError as e:
		# raise e
		return
def handle_goal_point(msg,name):
	g_pix_x = int(350 + msg.x * 35)
	g_pix_y = int(350 - msg.y * 35)
	pygame.draw.circle(pygame.display.get_surface(), (0, 255, 0), (g_pix_x, g_pix_y), 6, 0)
	pygame.display.update()

def handle_sampled_point(msg,name):
	global end_l_one,end_l_two,end_l_three
	p_pix_x = int(350 + msg.x * 35)
	p_pix_y = int(350 - msg.y * 35)
	fg = 0, 0, 0
	font = pygame.font.Font(None, 20)
	text = "%.2f ,%.2f"% (msg.x,msg.y)
    
	ren = font.render(text, 0, fg)
	if(name == "uav1"):
		pygame.draw.circle(pygame.display.get_surface(), (133, 0, 113), (p_pix_x, p_pix_y), 6, 0)
		if msg.z != 99 :
			pygame.draw.line(pygame.display.get_surface(), (235, 231, 0), end_l_one, (p_pix_x, p_pix_y), 2)
		end_l_one = (p_pix_x,p_pix_y)
	elif(name == "uav2"):
		pygame.draw.circle(pygame.display.get_surface(), (35, 170, 200), (p_pix_x, p_pix_y), 6, 0)
		if msg.z != 99 :
			pygame.draw.line(pygame.display.get_surface(), (235, 231, 0), end_l_two, (p_pix_x, p_pix_y), 2)
		end_l_two = (p_pix_x,p_pix_y)
	elif(name == "uav3"):
		pygame.draw.circle(pygame.display.get_surface(), (195, 108, 124), (p_pix_x, p_pix_y), 6, 0)
		if msg.z != 99:
			pygame.draw.line(pygame.display.get_surface(), (235, 231, 0), end_l_three, (p_pix_x, p_pix_y), 2)
		end_l_three = (p_pix_x,p_pix_y)

	S.blit(ren,(p_pix_x,p_pix_y))
	pygame.display.flip()

if __name__ == '__main__':
	rospy.init_node('MapTree_visual')
	S.fill((255, 255, 255))
	rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, handle_model_states)
	rospy.Subscriber('/uav1/sampled_point', hector_uav_msgs.msg.Vector, handle_sampled_point,"uav1")
	rospy.Subscriber('/uav2/sampled_point', hector_uav_msgs.msg.Vector, handle_sampled_point,"uav2")
	rospy.Subscriber('/uav3/sampled_point', hector_uav_msgs.msg.Vector, handle_sampled_point,"uav3")
	rospy.Subscriber('/uav1/actual_uav_goal', hector_uav_msgs.msg.Vector, handle_goal_point, "uav1")
	rospy.Subscriber('/uav2/actual_uav_goal', hector_uav_msgs.msg.Vector, handle_goal_point, "uav2")
	rospy.Subscriber('/uav3/actual_uav_goal', hector_uav_msgs.msg.Vector, handle_goal_point,"uav3")
	pygame.display.set_caption('Motion Planning Visualization')
	pygame.display.flip()

	running = True
	while not rospy.is_shutdown() and running:
		if total_points == 300:
			rospy.loginfo('Sampling is complete.')
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False

	rospy.spin()
