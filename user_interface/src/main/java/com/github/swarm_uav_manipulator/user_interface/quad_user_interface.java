package com.github.rosjava.swarm_uav_manipulator.user_interface;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import geometry_msgs.Point;
import javafx.application.Application;
import javafx.stage.Modality;
import javafx.stage.Stage;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.scene.text.Text;
import javafx.event.ActionEvent;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.geometry.Point3D;


public class quad_user_interface extends AbstractNodeMain {
  private volatile boolean sent = false;

  static Publisher<geometry_msgs.Point> ultimate_goal_pub_uav1;
  static Publisher<geometry_msgs.Point> ultimate_goal_pub_uav2;
  static Publisher<geometry_msgs.Point> ultimate_goal_pub_uav3;
  static Publisher<geometry_msgs.Point> ultimate_goal_pub_uav4;

  
  public static class GUI extends Application{
    	
    	public static final int UAV_COUNT = 4;
    	public static final float X_MIN = -10;
    	public static final float X_MAX = 10;
    	public static final float Y_MIN = -10;
    	public static final float Y_MAX = 10;
    	public static final float Z_MIN = 0;
    	public static final float Z_MAX = 4;
    	
    	
    	public GUI(){
    	}
    	
    	
    	@Override
    	public void start(Stage primaryStage) throws Exception{
    		primaryStage.setTitle("Quadrotor Goal Interface");
    		
		final geometry_msgs.Point _goal1 = ultimate_goal_pub_uav1.newMessage();
		final geometry_msgs.Point _goal2 = ultimate_goal_pub_uav2.newMessage();
		final geometry_msgs.Point _goal3 = ultimate_goal_pub_uav3.newMessage();
		final geometry_msgs.Point _goal4 = ultimate_goal_pub_uav4.newMessage();

    		GridPane grid = new GridPane();
    		grid.setAlignment(Pos.CENTER);
    		grid.setHgap(5);
    		grid.setVgap(5);
    		grid.setPadding(new Insets(25,25,25,25));
    		for(int i=1;i<=UAV_COUNT;i++){
    			final int inner_i = new Integer(i);
    			
    			Text scene_title = new Text("UAV - "+i+" Goal Position");
    			scene_title.setFont(Font.font("Arial",FontWeight.NORMAL,15));
    			grid.add(scene_title, 0,(i-1)*(UAV_COUNT+1),2, 1);
    			
    			Label uav_x = new Label("x :");
    			grid.add(uav_x,0,1 + (i-1)*(UAV_COUNT+1));
    			final TextField XCoordinateField = new TextField();
    			XCoordinateField.setPrefColumnCount(8);
    			grid.add(XCoordinateField, 1, 1 + (i-1)*(UAV_COUNT+1));
    			
    			Label uav_y = new Label("y :");
    			grid.add(uav_y,0,2 + (i-1)*(UAV_COUNT+1));
    			final TextField YCoordinateField = new TextField();
    			YCoordinateField.setPrefColumnCount(8);
    			grid.add(YCoordinateField, 1, 2 + (i-1)*(UAV_COUNT+1));
    			
    			Label uav_z = new Label("z :");
    			grid.add(uav_z,0, 3 + (i-1)*(UAV_COUNT+1));
    			final TextField ZCoordinateField = new TextField();
    			ZCoordinateField.setPrefColumnCount(8);
    			grid.add(ZCoordinateField, 1, 3 + (i-1)*(UAV_COUNT+1));
    			/*StackPane layout = new StackPane();
    			layout.getChildren().add(button);*/
    			Button btn = new Button("Send goal");
    			btn.setOnAction(new EventHandler<ActionEvent> (){
    				@Override
    				public void handle(ActionEvent event){
    					/*Code block for exception handling.*/
    					FlowPane pane = new FlowPane();
    				    pane.setVgap(10);
    				    pane.setAlignment(Pos.CENTER);
    				 
    				    Scene popup = new Scene(pane,400,200);
    				    
    				    Stage newStage = new Stage();
    			        newStage.setScene(popup);
    			        //tell stage it is meannt to pop-up (Modal)
    			        newStage.initModality(Modality.APPLICATION_MODAL);
    			        newStage.setTitle("Invalid coordinates");
    			        /*----------------------------------*/
    					float x,y,z;
    					try{
    						x = Float.parseFloat(XCoordinateField.getText());
    						y = Float.parseFloat(YCoordinateField.getText());
    						z = Float.parseFloat(ZCoordinateField.getText());
    					}
    					catch(Exception ex){
    						
    						empty_box_usage(pane);
    						newStage.show();
    						return;
    					}
    					if(x<X_MIN ||x >X_MAX  || y<Y_MIN || y>Y_MAX || z<Z_MIN || z>Z_MAX){
    						wrong_box_usage(pane);
    						newStage.show();
    						return;
    					}
    					
    					
    					switch(inner_i){
    						case 1:
    							_goal1.setX(x);
							_goal1.setY(y);
							_goal1.setZ(z);
							ultimate_goal_pub_uav1.publish(_goal1);
    							break;
    						case 2:
    							_goal2.setX(x);
							_goal2.setY(y);
							_goal2.setZ(z);
							ultimate_goal_pub_uav2.publish(_goal2);
    							break;
    						case 3:
    							_goal3.setX(x);
							_goal3.setY(y);
							_goal3.setZ(z);
							ultimate_goal_pub_uav3.publish(_goal3);
    							break;
    						case 4:
    							_goal4.setX(x);
							_goal4.setY(y);
							_goal4.setZ(z);
							ultimate_goal_pub_uav4.publish(_goal4);
    							break;
    					}
    				}
    			});
    			HBox hbBtn = new HBox(10);
    			hbBtn.setAlignment(Pos.BOTTOM_RIGHT);
    			hbBtn.getChildren().add(btn);
    			grid.add(hbBtn, 1,4 + (i-1)*(UAV_COUNT+1));
    			
    			
    		}
    		Scene scene = new Scene(grid,800,600);
    		primaryStage.setScene(scene);
    		primaryStage.show();
    		
    	}
    	private void empty_box_usage(FlowPane pane) {
    		
    		Text content = new Text("All fields must be filled!");
    		content.setFont(Font.font("Times New Roman",FontWeight.NORMAL,13));
    		pane.getChildren().add(content);
    	}
    	
    	private void wrong_box_usage(FlowPane pane){
    		Text content = new Text("Coordinates must not violate the boundary restrictions.\n"
    				+ "X => [" + X_MIN +"," + X_MAX + "]\n"
    				+ "Y => [" + Y_MIN +"," + Y_MAX + "]\n"
    				+ "Z => [" + Z_MIN +"," + Z_MAX + "]\n");
    		content.setFont(Font.font("Times New Roman",FontWeight.NORMAL,13));
    		pane.getChildren().add(content);
    		
    	}
    	
  }
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("quad_ui");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    ultimate_goal_pub_uav1 =
        connectedNode.newPublisher("/uav1/actual_uav_goal", geometry_msgs.Point._TYPE);
    ultimate_goal_pub_uav2 =
        connectedNode.newPublisher("/uav2/actual_uav_goal", geometry_msgs.Point._TYPE);
    ultimate_goal_pub_uav3 =
        connectedNode.newPublisher("/uav3/actual_uav_goal", geometry_msgs.Point._TYPE);
    ultimate_goal_pub_uav4 =
        connectedNode.newPublisher("/uav4/actual_uav_goal", geometry_msgs.Point._TYPE);


    Application.launch(GUI.class, null);
    
  }
}

