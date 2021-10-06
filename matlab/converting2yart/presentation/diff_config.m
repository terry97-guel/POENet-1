addpath ../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/

ccc

view_info = [80,16];
set_fig(figure(1),'pos',[0.4,0.4,0.3,0.55],...
    'view_info',view_info,'axis_info',5*[-1,+1,-1,+1,-1,+1],'AXIS_EQUAL',1,'GRID_ON',1,...
    'REMOVE_MENUBAR',1,'USE_DRAGZOOM',1,'SET_CAMLIGHT',1,'SET_MATERIAL','METAL',...
    'SET_AXISLABEL',1,'afs',18);

% Initialize a kinematic chain1
chain1 = init_chain('name','kinematic_chain');

% Add joint to the chain1
chain1 = add_joint_to_chain(chain1,'name','world');
chain1 = add_joint_to_chain(chain1,'name','J1','parent_name','world',...
    'p_offset',cv([0,0,0]),'a',cv([0,0,1]));
chain1 = add_joint_to_chain(chain1,'name','J2','parent_name','J1',...
    'p_offset',cv([1,0,0]),'a',cv([0,0,1]));
chain1 = add_joint_to_chain(chain1,'name','J3','parent_name','J2',...
    'p_offset',cv([1,0,0]),'a',cv([0,0,1]));
chain1 = add_joint_to_chain(chain1,'name','EE','parent_name','J3',...
    'p_offset',cv([1,0,0]),'a',cv([0,0,0]));

% Add link to the chain1
box_added = struct('xyz_min',[-2,-2,0],'xyz_len',[4,4,0.1],...
    'p_offset',cv([0,0,0]),'R_offset',rpy2r([0,0,0]*D2R),...
    'color',0.3*[1,1,1],'alpha',0.5,'ec','k');
chain1 = add_link_to_chain(chain1,'name','base_link','joint_name','world','box_added',box_added);

cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,0]),rpy2r([0,pi/2,0])),'radius',0.2,'height',1.0);
chain1 = add_link_to_chain(chain1,'name','L1','joint_name','J1','capsule',cap);
cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,0]),rpy2r([0,pi/2,0])),'radius',0.2,'height',1);
chain1 = add_link_to_chain(chain1,'name','L2','joint_name','J2','capsule',cap);
cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,0]),rpy2r([0,pi/2,0])),'radius',0.2,'height',1);
chain1 = add_link_to_chain(chain1,'name','L3','joint_name','J3','capsule',cap);


% Initialize a kinematic chain2
chain2 = init_chain('name','kinematic_chain');

% Add joint to the chain2
chain2 = add_joint_to_chain(chain2,'name','world');
chain2 = add_joint_to_chain(chain2,'name','J1','parent_name','world',...
    'p_offset',cv([0,0,0]),'a',cv([0,0,1]));
chain2 = add_joint_to_chain(chain2,'name','J2','parent_name','J1',...
    'p_offset',cv([1,0,1/tan(pi/6)]),'a',cv([0,0,1]));
chain2 = add_joint_to_chain(chain2,'name','J3','parent_name','J2',...
    'p_offset',cv([1,0,1/tan(pi/3)]),'a',cv([0,0,1]));
chain2 = add_joint_to_chain(chain2,'name','EE','parent_name','J3',...
    'p_offset',cv([1,0,-tan(pi/6)-tan(pi/3)]),'a',cv([0,0,0]));

% Add link to the chain2
box_added = struct('xyz_min',[-2,-2,0],'xyz_len',[4,4,0.1],...
    'p_offset',cv([0,0,0]),'R_offset',rpy2r([0,0,0]*D2R),...
    'color',0.3*[1,1,1],'alpha',0.5,'ec','k');
chain2 = add_link_to_chain(chain2,'name','base_link','joint_name','world','box_added',box_added);

cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,1/2/tan(pi/6)]),rpy2r([0,pi/6,0])),'radius',0.2,'height',1/sin(pi/6));
chain2 = add_link_to_chain(chain2,'name','L1','joint_name','J1','capsule',cap);
cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,1/2/tan(pi/3)]),rpy2r([0,pi/3,0])),'radius',0.2,'height',1/sin(pi/3));
chain2 = add_link_to_chain(chain2,'name','L2','joint_name','J2','capsule',cap);
cap   = get_capsule_shape('T_offset',pr2t(cv([0.5,0,-1/2/tan(pi/3)-1/2/tan(pi/6)]),rpy2r([0,pi/2+1.1622,0])),'radius',0.2,'height',1/sin(pi/2+1.1622));
chain2 = add_link_to_chain(chain2,'name','L3','joint_name','J3','capsule',cap);


traj1 = zeros(1e3,3);
traj2 = zeros(1e3,3);


% Loop
tick = 0; max_tick = 1e3; run_mode = 'STOP'; tfc = 'k';
while 1 % loop
    
    if isequal(run_mode,'RUN') % run something
        tick = tick + 1;
        joints2ctrl = {'J1','J2','J3'};
        q = sin(tick/10)*ones(length(joints2ctrl),1)*90*D2R;
        chain1 = update_chain_q(chain1,joints2ctrl,q,'FK',1,'FV',1);
        chain2 = update_chain_q(chain2,joints2ctrl,q,'FK',1,'FV',1);
        
        traj1(tick,:) = chain1.joint(5).p;
        traj2(tick,:) = chain2.joint(5).p;
    else
        pause(1e-6);
    end
    
    
    % Animate
    if mod(tick,1) == 0
        fig1 = plot_chain(chain1,'fig_idx',1,'subfig_idx',1,'fig_pos',[0.5,0.25,0.45,0.6],...
            'view_info',[68,16],'axis_info',[-2.5,+2.5,-2.5,+2.5,0,+3.5],'USE_ZOOMRATE',1,...
            'PLOT_LINK',1,'llc','k','llw',1,'lls','-',...
            'PLOT_BOX_ADDED',1,'PLOT_CAPSULE',1,'cfc','','cfa',0.2,...
            'PLOT_COM',1,'csc','r','csr',0.05,'csa',0.5,...
            'PLOT_JOINT_AXIS',1,'jal',0.1,'jalw',2,'jals','-',...
            'PLOT_JOINT_SPHERE',0,'jsr',0.05,'jsfc','k','jsfa',0.75,...
            'PLOT_ROTATE_AXIS',1,'ral',0.3,'rac','','raa',0.75,...
            'PLOT_JOINT_NAME',1, ...
            'PLOT_JOINT_V',0,'jvfc','m','jvfa',0.7,'jvar',0.05,'jvsw',0.05,'jvtw',0.1, ...
            'PLOT_JOINT_W',0,'jwfc','c','jwfa',0.7,'jwar',0.05,'jwsw',0.05,'jwtw',0.1, ...
            'PLOT_LINK_V',0,'lvfc','m','lvfa',0.7,'lvar',0.05,'lvsw',0.05,'lvtw',0.1, ...
            'PLOT_LINK_W',0,'lwfc','c','lwfa',0.7,'lwar',0.05,'lwsw',0.05,'lwtw',0.1 ...
            );
        plot_traj(traj1,'fig_idx',1,'subfig_idx',2,'tlc','k');
        
        fig2 = plot_chain(chain2,'fig_idx',1,'subfig_idx',3,'fig_pos',[0.5,0.25,0.45,0.6],...
            'view_info',[68,16],'axis_info',[-2.5,+2.5,-2.5,+2.5,0,+3.5],'USE_ZOOMRATE',1,...
            'PLOT_LINK',1,'llc','k','llw',1,'lls','-',...
            'PLOT_BOX_ADDED',1,'PLOT_CAPSULE',1,'cfc','','cfa',0.2,...
            'PLOT_COM',1,'csc','r','csr',0.05,'csa',0.5,...
            'PLOT_JOINT_AXIS',1,'jal',0.1,'jalw',2,'jals','-',...
            'PLOT_JOINT_SPHERE',0,'jsr',0.05,'jsfc','k','jsfa',0.75,...
            'PLOT_ROTATE_AXIS',1,'ral',0.3,'rac','','raa',0.75,...
            'PLOT_JOINT_NAME',1, ...
            'PLOT_JOINT_V',0,'jvfc','m','jvfa',0.7,'jvar',0.05,'jvsw',0.05,'jvtw',0.1, ...
            'PLOT_JOINT_W',0,'jwfc','c','jwfa',0.7,'jwar',0.05,'jwsw',0.05,'jwtw',0.1, ...
            'PLOT_LINK_V',0,'lvfc','m','lvfa',0.7,'lvar',0.05,'lvsw',0.05,'lvtw',0.1, ...
            'PLOT_LINK_W',0,'lwfc','c','lwfa',0.7,'lwar',0.05,'lwsw',0.05,'lwtw',0.1 ...
            );
        plot_traj(traj2,'fig_idx',1,'subfig_idx',4,'tlc','k');
        
        title_str = sprintf('[%s] Tick:[%d] ([r]:run [s]:stop [q]:quit)',...
            run_mode,tick);
        plot_title(title_str,'fig_idx',1,'tfs',20,'tfc',tfc);
        drawnow;
        if ~ishandle(fig2)
            break; 
        end
%         if (~ishandle(fig1) || ~ishandle(fig2)) 
%             break; 
%         end
    end
    
    % Keyboard handler
    if ~isempty(g_key) % if key pressed
        switch g_key
            case 'q'                % press 'q' to quit
                break;
            case 's'                % press 's' to stop
                run_mode = 'STOP';
                tfc      = 'k';
            case 'r'                % press 'r' to run
                run_mode = 'RUN';
                tfc      = 'b';
        end
        g_key = ''; % reset key pressed
    end % if ~isempty(g_key) % if key pressed
    
    % Terminate condition
    if tick > max_tick
        break;
    end
    
end % for tick = 1:max_tick % loop
if ishandle(fig2), plot_title('Terminated','fig_idx',1,'tfs',20,'tfc','r'); end
fprintf('Done.\n');
