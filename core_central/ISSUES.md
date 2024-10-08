# Gazebo Issues

## Immediate crash

Sometimes gazebo will immediately crash with the following error message. Just restart it, it doesn't happen very often.
              gazebo: X Error of failed request:  GLXBadDrawable
              gazebo:   Major opcode of failed request:  154 (GLX)
              gazebo:   Minor opcode of failed request:  5 (X_GLXMakeCurrent)
              gazebo:   Serial number of failed request:  49
              gazebo:   Current serial number in output stream:  49
              gazebo: terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
              gazebo:   what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
              gazebo: Aborted (core dumped)
              gazebo: gazebo exited with status 134
              gazebo: Could not remove process working directory '/tmp/rosmon-node-Afm1Aj' after process exit: Directory not empty

# PX4 Issues

## ID problems

While trying to spawn multiple uavs in a gazebo I found that you must spawn a uav with ID 0, then ID 1, etc. If you start at 1, for example, instead of 0 the uav will not get initialized correctly and gazebo will stop responding, but not actually crash and even Ubuntu doesn't come up with the normal "Force Quit/Wait" dialog box that it does when gazebo crashes.
<node name="sitl_$(arg ID)" pkg="px4" type="px4" output="screen"
	  args="$(find px4)/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -i $(arg ID) $(arg px4_command_arg1)">
    </node>

Also, when launching mavros the tgt_system parameter should be 1 + the ID passed to the node above. As in, <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>.


## Sensitive to physics parameters

If the real_time_update_rate parameter isn't set to 250, the px4 software will fail an assertion and crash gazebo. If you just set the real_time_update_rate to 250 but don't include the rest of the parameters shown below, gazebo will still crash. It seems necessary to add/replace the physics tag in your gazebo world file with the following for the px4 to work correctly:

<physics name='default_physics' default='0' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>