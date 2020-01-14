classdef LM_Proposal < handle
         
         properties  
             
          %------------/module_&_debugging_parameters-------------------------
           DBG_new_landmarks  %[Debug Only] a structure that contains new landmarks  
           CTRL_MAX_Landmarks= 5  %[Module Parameter] controls the max number of landmarks to be added at each epoch, 5 by default
           INITIAL_LOCATIONS
           
           LMP_n_L_k % number of expected extracted landmarks
           LMP_n_k   % number of expected measurements
           LMP_H_k   % Jacobian
           LMP_H_k_new   % Jacobian with new lms added
           LMP_B_j   % non-faulted msmts extraction_matrix
           LMP_V     % Just a normal-looking measurement cov matrix
           LMP_P_F_M
           LMP_n_M
           LMP_n_L_M  
           M
           LMP_n_total % number of msmts
           LMP_m_M %states to estimate
           lmp_alpha % the updated_extract state of interest vector
           LMP_abs_msmt_ind
           LMP_PX_M
           LMP_inds_H
           LMP_sigma_hat
           LMP_Lambda
           LMP_p_hmi
           LMP_n_H
           LMP_P_H
           A_aug
           C_req
           LM_abs_msmt_ind
           m
           B_j

         end
         
         methods
             function obj = LM_Proposal(im,params,estimator,debug)

              if(debug == 1)
                 p_phmi = calculate_phmi_pseudo(obj,im,params,estimator,obj.DBG_new_landmarks)
              else
                %%  ---------under costruction------------
                 for lm_i = 1:obj.CTRL_MAX_Landmarks % try different landmark number until it reaches the max
                    %---------------------------------
                     % construct the initial condition
                     new_landmarks_0 = [];
                     tmp_map = estimator.landmark_map;
                     for x0_i = 1:lm_i
                         %------optimize on separateness-----
                         rand_dir=(rand*2)-1;
                         init_init = [estimator.x_true(1)+(params.lidarRange-8)*cos(2*pi*x0_i/lm_i+rand_dir*pi),...
                              estimator.x_true(2)+(params.lidarRange-8)*sin(2*pi*x0_i/lm_i+rand_dir*pi)];
                         [init_pose,distance] = fmincon(@(posi) find_farest_point(posi,tmp_map),init_init,...
                             [],[],[],[],[],[],...
                             @(posi) constraints(obj,im,params,estimator,[estimator.x_true(1),estimator.x_true(2),estimator.x_true(3)],posi));
                         tmp_map = [tmp_map;init_pose];   
                         obj.INITIAL_LOCATIONS = [obj.INITIAL_LOCATIONS; init_pose];
                         %-----------------------------------     
                         
                         new_landmarks_0 = [new_landmarks_0;init_pose];%...
                             %estimator.x_true(1)+(params.lidarRange-2)*cos(2*pi*x0_i/lm_i+rand_dir*pi),...
                             %estimator.x_true(2)+(params.lidarRange-2)*sin(2*pi*x0_i/lm_i+rand_dir*pi)]
                     end     
                     
                    %opt====!!!IMPORTANT!!!===============================
                     para_x0 = new_landmarks_0;
                     para_A = [];
                     para_b = [];
                     para_Aeq = [];
                     para_beq = [];
                     para_lb = [];
                     para_ub = [];

                     [new_landmark_return,ps_phmi_out] = fmincon(@(new_landmarks) calculate_phmi_pseudo(obj,im,params,estimator,new_landmarks),... 
                         para_x0,para_A,para_b,para_Aeq,para_beq,para_lb,para_ub,...
                         @(new_landmarks) constraints(obj,im,params,estimator,[estimator.x_true(1),estimator.x_true(2),estimator.x_true(3)],new_landmarks));
                         %@(new_landmarks) constraint_in_detection_range(params,[estimator.x_true(1),estimator.x_true(2)],new_landmarks ) );
                     if( (ps_phmi_out/1e8) <im.p_hmi)
                         im.p_hmi = ps_phmi_out/1e8;
                         estimator.n_k= estimator.n_k + size(new_landmark_return,1)*params.m_F;
                         estimator.H_k= [estimator.H_k;obj.LMP_H_k_new];
                         estimator.landmark_map = [estimator.landmark_map;new_landmark_return];
                         estimator.num_landmarks= size(estimator.landmark_map, 1);
                         params.landmark_map = estimator.landmark_map;
                         im.A=obj.A_aug;
                         im.Gamma_fg= im.A' * im.A;
                         im.PX_M= inv(im.Gamma_fg);
                         estimator.PX= im.PX_M( end - params.m + 1 : end, end - params.m + 1 : end );
                         im.PX_prior= im.PX_M( params.m + 1 : 2*params.m, params.m + 1 : 2*params.m );
                         im.Gamma_prior= inv(im.PX_prior);
                         im.n_L_M= obj.LMP_n_L_M;
                         estimator.n_L_M=im.n_L_M;
                         im.n_M=obj.LMP_n_M;
                         im.n_total=obj.LMP_n_total;
                         im.modified_q = true;
                         break;
                     else
                         im.modified_q = false;
                     end
                 end           
              end
                 
             end
          %================================================================
          %================================================================
          function lmp_p_hmi_op = calculate_phmi_pseudo(obj,im,params,estimator,new_landmarks)
                 obj.M= im.M; 
                 obj.C_req= params.continuity_requirement;
                 obj.LMP_abs_msmt_ind = im.abs_msmt_ind;
                 obj.m= im.m;
                 obj.LMP_n_M= estimator.n_k + size(new_landmarks,1)*params.m_F + sum( im.n_ph(1:obj.M - 1) );
                  % number of landmarks over the horizon
                 obj.LMP_n_L_M= obj.LMP_n_M / params.m_F; 
                  % compute extraction vector
                 lmp_alpha= obj.build_state_of_interest_extraction_matrix(params, estimator.x_true);
                  % total number of msmts (prior + relative + abs)
                 obj.LMP_n_total= obj.LMP_n_M + (obj.M + 1) * (params.m);
                  % number of states to estimate
                 % obj.m_M= (obj.M + 1) * params.m;
                 obj.A_aug = build_augmented_whiten_jacobian_A(obj,im,params,estimator,new_landmarks);
                 
                 ind= size(im.A,1);
                 for i=1:size(new_landmarks,1)
                     tmp= [ind+1;ind+2];
                     ind= ind+2;
                     obj.LMP_abs_msmt_ind= [obj.LMP_abs_msmt_ind,tmp];
                 end
                 obj.LMP_Lambda= obj.A_aug'*obj.A_aug; % provides the lambda
                 obj.LMP_PX_M= inv(obj.LMP_Lambda);
                 obj.LMP_P_F_M= ones(obj.LMP_n_L_M, 1) * params.P_UA;
                 obj.LMP_n_H= obj.LMP_n_L_M;
                 % initialization of p_hmi
                 lmp_p_hmi_op=0;
                 if obj.LMP_n_M < params.m + params.m_F
                      % if we don't have enough landmarks --> P(HMI)= 1
                     lmp_p_hmi_op= 1;
                 else
                      % standard deviation in the state of interest
                     obj.LMP_sigma_hat= sqrt( (lmp_alpha' / obj.LMP_Lambda) * lmp_alpha );
                     
                     for i= 0:obj.LMP_n_H   
                         % compute P(HMI | H) for the worst-case fault
                         LMP_p_hmi_H= obj.compute_p_hmi_H(lmp_alpha, i, params);
 
                         % Add P(HMI | H) to the integrity risk
                         if i == 0
                             lmp_p_hmi_op= lmp_p_hmi_op + LMP_p_hmi_H * prod( 1 - obj.LMP_P_F_M );
                         else
                             lmp_p_hmi_op= lmp_p_hmi_op + LMP_p_hmi_H * params.P_UA;
                         end
                     end
                 end
                 lmp_p_hmi_op=lmp_p_hmi_op*1e8;
          end
          %----------------------------------------------------------------
          function A_aug = build_augmented_whiten_jacobian_A(obj,im,params,estimator,new_landmarks)
                 num_lm_prp = size(new_landmarks,1);
                 compute_lidar_H_k_new(obj,estimator, params, new_landmarks);
                 A_old = im.A;
                 obj.LMP_V = kron( eye(num_lm_prp) , params.sqrt_inv_R_lidar );
                 A_new = [zeros(size(new_landmarks,1)*params.m_F,size(A_old,2)-obj.m),obj.LMP_V*obj.LMP_H_k_new];
                 A_aug = [A_old;A_new];
          end
          [cst,ceq] = constraints(obj,im,params,estimator,x_true,new_landmarks);
         end
        
end