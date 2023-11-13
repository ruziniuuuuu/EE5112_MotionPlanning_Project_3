% dtheta: estimated gradient
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);
% variable declaration
dtheta = zeros(nJoints, nDiscretize);

%% TODO: iterate over all joints to compute dtheta: (complete your code according to the STOMP algorithm)
for joint = 1:nJoints
    % Iterate over all time steps
    for t = 1:nDiscretize
        % Compute the gradient for the current joint and time step
        gradient_sum = 0;
        
        % Iterate over all samples
        for sample = 1:size(em{joint}, 1)
            % Compute the difference between the current trajectory probability
            % and the sampled trajectory probability
            diff_prob = trajProb(sample, t) - em{joint}(sample, t);
            
            % Accumulate the gradient contribution from each sample
            gradient_sum = gradient_sum + diff_prob * em{joint}(sample, t);
        end
        
        % Update the gradient for the current joint and time step
        dtheta(joint, t) = gradient_sum / size(em{joint}, 1);
    end
end