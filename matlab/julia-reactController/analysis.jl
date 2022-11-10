using Plots
# using PlotlyJS
using CSV
using DataFrames
using LinearAlgebra
using Rotations
using Statistics
using NaNStatistics
using OrderedCollections
using StatsBase
using SavitzkyGolay
using StatsPlots
using CategoricalArrays

plotlyjs(size=(1000,1000), legend=true, markerstrokewidth=0,markersize=2)
gr(size=(1000,1000), legend=true, markerstrokewidth=0,markersize=2, fmt = :png)
pos = findfirst((x)->(x=="image/svg+xml"), VSCodeServer.DISPLAYABLE_MIMES)
if !isnothing(pos)
    popat!(VSCodeServer.DISPLAYABLE_MIMES, pos)
    println("Popped!")
end

function prepareTimeData(d, cart=false)
    solverExitCode = 2+3
    timeToSolveProblem_s = 3+3
    t = d[:,3].-d[1,3]
    time_diff = diff(t)
    replace!(x -> x >0.1 ? NaN : x, time_diff)
    replace!(x -> x <0.002 ? NaN : x, time_diff)
    if (cart)
        return (t=t, time_diff=time_diff)
    end
    
    return (t=t, time_diff=time_diff, solver_time=d[:,timeToSolveProblem_s], solver_exit=d[:,solverExitCode])
end

vecnorm_col(A) = [norm(@view A[:,i]) for i=1:size(A,2)]
vecnorm_row(A) = [norm(@view A[i,:]) for i=1:size(A,1)]

function prepareArmData(t, d, indexes, posThr, oriThr, dT, exper)
    data = Dict()
    data["pos_error"] = vecnorm_row(d[:, indexes.x_n]-d[:, indexes.x_t])
    data["refpos_error"] = vecnorm_row(d[:, indexes.x_d]-d[:, indexes.x_t])
    
    data["pose_diff"] = zeros(size(d)[1],)
    data["refpose_diff"] = zeros(size(d)[1],)
    data["theta"] = zeros(size(d)[1],)
    data["reftheta"] = zeros(size(d)[1],)
    
    for i = 1:size(d)[1]
        ori_des = RotMatrix(AngleAxis(d[i, indexes.o_n[end]], d[i, indexes.o_n[1]], d[i, indexes.o_n[2]], d[i, indexes.o_n[3]]))
        ori_final = RotMatrix(AngleAxis(d[i, indexes.o_d[end]], d[i, indexes.o_d[1]], d[i, indexes.o_d[2]], d[i, indexes.o_d[3]]))
        ori_cur = RotMatrix(AngleAxis(d[i, indexes.o_t[end]], d[i, indexes.o_t[1]], d[i, indexes.o_t[2]], d[i, indexes.o_t[3]]))
        Hr = Matrix{Float64}(I,4,4)
        Hfinal = Matrix{Float64}(I,4,4)
        He = Matrix{Float64}(I,4,4)
        Hr[1:3,1:3] = ori_des
        Hfinal[1:3,1:3] = ori_des
        He[1:3,1:3] = ori_cur
        Hr[1:3,4]=d[i, indexes.x_n]
        Hfinal[1:3,4]=d[i, indexes.x_d]
        He[1:3,4]=d[i, indexes.x_t]
        T = ori_des * transpose(ori_cur)
        Tfinal = ori_final * transpose(ori_cur)
        data["theta"][i] = rotation_angle(T)
        data["reftheta"][i] = rotation_angle(Tfinal)
        data["pose_diff"][i] = norm(Hr-He)
        data["refpose_diff"][i] = norm(Hfinal-He)
    end
    
    data["cart_vels"] = diff(d[:,indexes.x_t], dims=1)./dT
    data["cart_vels_norm"] = vecnorm_row(data["cart_vels"])
    data["cart_acc"] = diff(data["cart_vels"], dims=1)./dT
    data["cart_acc_norm"] = vecnorm_row(data["cart_acc"])
    data["cart_jerk"] = diff(data["cart_acc"], dims=1)./dT
    data["cart_jerk_norm"] = vecnorm_row(data["cart_jerk"])
    
    sgfilter = SGolay(25, 10, 0, 1) 
    sgfilter2 = SGolay(25, 10, 2, 1/dT) 
    
    data["joint_vels"] = d[:,indexes.qDot]
    data["joint_vels2"] = diff(d[:,indexes.qIntegrated], dims=1)./dT
    data["joint_vels4"] = diff(d[:,indexes.q], dims=1)./dT
    data["joint_vels3"] = similar(data["joint_vels4"])
    data["joint_jerk3"] = similar(data["joint_vels"])    
    for i = 1:size(data["joint_vels"],2)
        data["joint_vels3"][:,i] = sgfilter(data["joint_vels4"][:,i]).y
        data["joint_jerk3"][:,i] = sgfilter2(data["joint_vels"][:,i]).y
    end 
    data["joint_jerk"] = diff(diff(data["joint_vels3"], dims=1), dims=1)./dT^2 # diff(diff(data["joint_vels"], dims=1)./dT, dims=1)./dT
    data["joint_jerk2"] = diff(diff(data["joint_vels2"], dims=1), dims=1)./dT^2
    data["joint_vels_norm"] = vecnorm_row(d[:,indexes.qDot])
    data["joint_vels_norm2"] = vecnorm_row(data["joint_vels2"])
    data["joint_vels_norm3"] = vecnorm_row(data["joint_vels3"])
    data["joint_vels_norm4"] = vecnorm_row(data["joint_vels4"])
    data["joint_jerk_norm"] = vecnorm_row(data["joint_jerk"]) 
    data["joint_jerk_norm2"] = vecnorm_row(data["joint_jerk2"])
    data["joint_jerk_norm3"] = vecnorm_row(data["joint_jerk3"])
    data["distance"] = sum(abs.(diff(d[:,indexes.qIntegrated], dims=1)))

    lastpos_target = []
    time_to_reach = []
    time_to_reach_pos = []
    time_to_reach_ori = []
    new_target_array = []
    
    for i = 2:size(d,1) # change between targets
        if sum(abs.(d[i, indexes.x_d] - d[i-1, indexes.x_d])) > 0.001 && sum(abs.(d[i-1, indexes.x_d])) > 0.001
            push!(lastpos_target, i-1)
        end
    end
    push!(lastpos_target, size(d,1))
    idx = 1
    push!(new_target_array, 1)
    for i = lastpos_target
        while (idx <= size(d,1) && !(idx in lastpos_target))
            if (data["refpos_error"][idx] < posThr && data["reftheta"][idx] < oriThr)
                break
            end
            idx += 1
        end
        push!(time_to_reach, idx) 
        idx = i+1
        if (idx <= min(size(d,1), size(t,1)))
            push!(new_target_array, idx)
        end
    end
    idx = 1
    for i = lastpos_target
        while (idx <= size(d,1) && !(idx in lastpos_target))
            if data["refpos_error"][idx] < posThr
                break
            end
            idx += 1
        end
        push!(time_to_reach_pos, idx) 
        idx = i+1
    end
    idx = 1
    for i = lastpos_target
        while (idx <= size(d,1) && !(idx in lastpos_target))
            if data["reftheta"][idx] < oriThr
                break
            end
            idx += 1
        end
        push!(time_to_reach_ori, idx) 
        idx = i+1
    end
    data["target_lastindex"] = lastpos_target 
    data["time_to_reach"] = t[time_to_reach] .- t[new_target_array]
    data["time_to_reach_pos"] = t[time_to_reach_pos] .- t[new_target_array]
    data["time_to_reach_ori"] = t[time_to_reach_ori] .- t[new_target_array]
    data["ttr_idx"] = time_to_reach
    data["ttr_pos_idx"] = time_to_reach_pos
    data["ttr_ori_idx"] = time_to_reach_ori
    data["target_start_idx"] = new_target_array
    return data
end


function parseData(folder, exper, posThr, oriThr, dT, cart=false)
    if (cart)
        param_idx = nothing
        param_idx2 = nothing
    else
        params = CSV.read(folder*"param.log", DataFrame; header=0)

        param_idx = (part=params[1,1], chainDof=params[1,2], selCol=params[1,3], blim=params[1,4:2:4+params[1,2]*2-1], ulim=params[1,5:2:5+params[1,2]*2-1], vmax=params[1,4+params[1,2]*2], trajSpeed=params[1,5+params[1,2]*2], tol=params[1,6+params[1,2]*2], globalTol=params[1,7+params[1,2]*2], 
                    dT=params[1,8+params[1,2]*2], timeLimit=params[1,9+params[1,2]*2], stiff=params[1,10+params[1,2]*2], colPoints=params[1,(11:13).+params[1,2]*2], hitt=params[1,14+params[1,2]*2], restPos=params[1,15+params[1,2]*2], oriControl=params[1,16+params[1,2]*2], 
                    gaze=params[1,17+params[1,2]*2])
        dT = param_idx.dT
        param_idx2 = nothing
        if (size(params,2) > 38)
            param_idx2 = (part=params[1,39], chainDof=params[1,40], selCol=params[1,41], blim=params[1,42:2:42+params[1,40]*2-1], ulim=params[1,43:2:43+params[1,40]*2-1])
        end
    end
    df = CSV.read(folder*"data.log", DataFrame; header=0) |> Matrix{Float64}
    # df = df[1000:5000,:]
    skip_lines = 0
    end_lines = size(df,1)
    target_pos = cart ? (4:6) : (8:10)
    if exper == 3
        for i = 2:size(df,1) # change between targets
            if sum(abs.(df[i, target_pos] - df[i-1, target_pos])) > 0.001 && sum(abs.(df[i-1, target_pos])) > 0.001
                skip_lines = i-1
               break
            end
        end
    elseif exper == 4 || exper == 5
        start_pos = nothing #param_idx.part == "left" ? [-0.299,-0.074,0.1] : [-0.299,0.074,0.1]
        for i = 2:size(df,1) # change between targets
            if isnothing(start_pos)
                if sum(abs.(df[i, target_pos])) > 0.001
                    start_pos = df[i, target_pos]#
                end
            elseif sum(abs.(df[i, target_pos] - start_pos)) > 0.0001 && sum(abs.(df[i, target_pos])) > 0.001
                skip_lines = i-1
                break
            end
        end
        for i = skip_lines+10:size(df,1) # change between targets
            if sum(abs.(df[i, target_pos] - df[i-1, target_pos])) < 0.0001 && sum(abs.(df[i, target_pos] - df[i-10, target_pos])) < 0.0001
                end_lines = i-10
                break
            end
        end
    elseif exper == 1
        skip_lines = 501
        end_lines=1500
    end
    
    # end_lines = 710
    df = df[skip_lines+1:end_lines,:]
    data = df[:,4:end]


    general_data = prepareTimeData(df)
    
    if (cart)
        mArm = (x_d=1:3, x_t=4:6, x_n=7:9, o_d=10:13, o_t=14:17, o_n=18:21, qDot=22:31, qIntegrated=32:41, q=42:51);
        sArm = nothing
        sArm_data = nothing
        param_idx = nothing
        param_idx2 = nothing
    else
        mArm = (chainDOF=1, avoidance=4, x_d=5:7, x_t=8:10, x_n=11:13, o_d=14:17, o_t=18:21, o_n=22:25, qDot=26:35, q=36:45, qIntegrated=46:55, vLimAdapted=56:75);
        if (size(data)[2] > 77)
            sArm = (chainDOF=76, avoidance=77, x_d=78:80, x_t=81:83, x_n=84:86, o_d=87:90, o_t=91:94, o_n=95:98, qDot=99:108, q=109:118, qIntegrated=119:128, vLimAdapted=129:148);
            sArm_data = prepareArmData(general_data.t, data, sArm, posThr, oriThr, param_idx.dT, exper)
        else
            sArm = nothing
            sArm_data = nothing
        end
    end
    mArm_data = prepareArmData(general_data.t, data, mArm, posThr, oriThr, dT, exper)
  #  print(size(data))
    return mArm, mArm_data, sArm, sArm_data, data, general_data, param_idx, param_idx2, df[1,3], df[end,3] # start_time, end_time
end


function plotTargetErrors(t, data, mArm, mArm_data, exper, name)
    p1 = scatter(t, data[:,mArm.x_d[1]]; label="Target X", ylabel="Position [m]")
    scatter!(t, data[:,mArm.x_n[1]]; label="Next X")
    scatter!(t, data[:,mArm.x_t[1]]; label="Current X")

    p2 = scatter(t, data[:,mArm.x_d[2]]; label="Target Y", ylabel="Position [m]")
    scatter!(t, data[:,mArm.x_n[2]]; label="Next Y")
    scatter!(t, data[:,mArm.x_t[2]]; label="Current Y")

    p3 = scatter(t, data[:,mArm.x_d[3]]; label="Target Z", ylabel="Position [m]")
    scatter!(t, data[:,mArm.x_n[3]]; label="Next Z")
    scatter!(t, data[:,mArm.x_t[3]]; label="Current Z")
    p4 = scatter(t, mArm_data["pos_error"], color=:green, legend=:topleft, label="Position error", ylabel="Distance [m]", left_margin = 5Plots.mm, right_margin = 15Plots.mm)
    axis2 = twinx();
    scatter!(axis2, t,abs.(mArm_data["theta"]), color=:orange, legend=:topright, label="Orientation error", ylabel="Distance [rad]", left_margin = 5Plots.mm, right_margin = 15Plots.mm)
    p5 = scatter(t, mArm_data["refpos_error"], color=:green, legend=:topleft, label="Position error", ylabel="Distance [m]", left_margin = 5Plots.mm, right_margin = 15Plots.mm)
    axis2 = twinx();
    scatter!(axis2, t,abs.(mArm_data["reftheta"]), color=:orange, legend=:topright, label="Orientation error", ylabel="Distance [rad]", left_margin = 5Plots.mm, right_margin = 15Plots.mm)
    
    pt = plot(p1, p2, p3, p4, p5; layout=grid(5,1), size=(1000,1200), title=["$n" for j=1:1, n in ["X pos", "Y pos", "Z pos", "Next pos errors", "Next pos errors", "Target pos errors", "Target pos errors"]])
    y =ones(3)
    title = scatter(y, marker=0,markeralpha=0, annotations=(2, y[2], text("Target evalutation for Exp " *string(exper), font(18))), titlefontsize=18,axis=([], false), grid=false, leg=false,size=(200,100))
    p = plot(title, pt,layout=grid(2,1,heights=[0.01,0.99]))
    display(p)
    savefig(p,"targets_"* name * "_exp" * string(exper) * ".png")
    if exper == 1
        scatter3d(data[mArm_data["target_start_idx"],mArm.x_d[1]], data[mArm_data["target_start_idx"],mArm.x_d[2]], data[mArm_data["target_start_idx"],mArm.x_d[3]])
        pp = plot3d!([0,0], [0,0], [-0.5,0.5]; linewidth=10)
        display(pp)
        savefig(pp,"grid_"* name * "_exp" * string(exper) * ".png")
    end

end


function plotTargets(t, data, mArm, exper, name)
    if exper > 2
        p1 = plot(;xlabel="Y [m]", ylabel="Z [m]")
        scatter!(data[:,mArm.x_d[2]], data[:,mArm.x_d[3]]; label="Target pos")
        scatter!(data[:,mArm.x_n[2]], data[:,mArm.x_n[3]]; label="Next pos")
        scatter!(data[:,mArm.x_t[2]], data[:,mArm.x_t[3]]; label="Current pos")
    else
        p1 = plot3d(;camera=(60,0), xlabel="X [m]", ylabel="Y [m]", zlabel="Z [m]", aspect_ratio=:equal)      
        scatter3d!(data[:,mArm.x_d[1]], data[:,mArm.x_d[2]], data[:,mArm.x_d[3]]; markersize=6, label="Target pos")
        scatter3d!(data[:,mArm.x_n[1]], data[:,mArm.x_n[2]], data[:,mArm.x_n[3]]; label="Next pos")
        scatter3d!(data[:,mArm.x_t[1]], data[:,mArm.x_t[2]], data[:,mArm.x_t[3]]; label="Current pos")
    end
    display(p1)
    savefig(p1,"targets_"* name * "_exp" * string(exper) * ".png")

end

function plotJoints(t, data, names, ylab, name, suptitle=nothing, xline1=nothing, yline1=nothing, xline2=nothing, yline2=nothing)
    p1=plot(t, data;
    xlabel = "Time [s]",
    ylabel = ylab,
    layout = grid(2,5),
    linewidth = 3,
    title = ["$n" for j=1:1, n in names],
    legend = false,
    size=(1400,800)
    )
    if !isnothing(xline1)
        plot!(xline1, yline1; linewidth=2)
        plot!(xline2, yline2; linewidth=2) 
        if (size(xline1,1) == 2)
            qGuard = 0.25*0.1.*(yline2-yline1)

            plot!(xline1, yline1+qGuard; linewidth=2)
            plot!(xline1, yline1+2*qGuard; linewidth=2)
            plot!(xline2, yline2-2*qGuard; linewidth=2) 
            plot!(xline2, yline2-qGuard; linewidth=2) 
        end
    end
   
    if  !isnothing(suptitle)
        title = scatter(ones(3), marker=0,markeralpha=0, annotations=(2, 1, text(suptitle, font(18))), titlefontsize=18,axis=([], false), grid=false, leg=false,size=(200,100))
        p = plot(title, p1,layout=grid(2,1,heights=[0.01,0.99]))
    else
        p = plot(p1)
    end
    display(p)
    savefig(p, replace(suptitle, " " => "_") * "_" * name * ".png")
end


function createStatsDict()
    stats = Dict()
    stats["rmse_refpos"] = Vector{Float64}()
    stats["rmse_pos"] = Vector{Float64}()
    stats["rmse_refori"] = Vector{Float64}()
    stats["rmse_ori"] = Vector{Float64}()
    stats["cart_acc_max"] = Vector{Float64}()
    stats["cart_acc_sum"] = Vector{Float64}()
    stats["cart_acc_mean"] = Vector{Float64}()
    stats["cart_jerk_max"] = Vector{Float64}()
    stats["cart_jerk_sum"] = Vector{Float64}()
    stats["cart_jerk_median"] = Vector{Float64}()
    stats["cart_jerk_mean"] = Vector{Float64}()
    stats["joint_jerk_max"] = Vector{Float64}()
    stats["joint_jerk_sum"] = Vector{Float64}()
    stats["joint_jerk_median"] = Vector{Float64}()
    stats["joint_jerk_mean"] = Vector{Float64}()
    stats["dists"] = Vector{Float64}()
    stats["max_ttr"] = Vector{Float64}()
    stats["med_ttr"] = Vector{Float64}()
    stats["mean_ttr"] = Vector{Float64}()
    stats["meanpos_ttr"] = Vector{Float64}()
    stats["meanori_ttr"] = Vector{Float64}()
    stats["stdpos_ttr"] = Vector{Float64}()
    stats["stdori_ttr"] = Vector{Float64}()
    stats["num_ttr"] = Vector{Float64}()
    stats["time_ttr"] = Vector{Vector{Float64}}()
    stats["time_pos_ttr"] = Vector{Vector{Float64}}()
    stats["time_ori_ttr"] = Vector{Vector{Float64}}()
    stats["pos_ttr"] = Vector{Vector{Float64}}()
    stats["pos_ttr2"] = Vector{Vector{Float64}}()
    stats["ori_ttr"] = Vector{Vector{Float64}}()
    stats["ori_ttr2"] = Vector{Vector{Float64}}()
    stats["joint_jerk_norm"] = Vector{Vector{Float64}}()
    stats["cart_jerk_norm"] = Vector{Vector{Float64}}()
    stats["cart_acc_norm"] = Vector{Vector{Float64}}()
    stats["refpos_error"] = Vector{Vector{Float64}}()
    stats["refori_error"] = Vector{Vector{Float64}}()
    stats["obs_dist"] = Vector{Vector{Float64}}()
    stats["pos_error"] = Vector{Vector{Float64}}()
    stats["ori_error"] = Vector{Vector{Float64}}()
    stats["reached_targets"] = Vector{Vector{Bool}}()
    stats["reached_pos"] = Vector{Vector{Bool}}()
    stats["reached_ori"] = Vector{Vector{Bool}}()
    return stats
end


function analyzeData(f_name, folders, expers, posThr, oriThr, dT, show_plots=false)
    multistats = OrderedDict()
    f_name = f_name * "/"
    # 95 percent CI je 1.96* std/N
    for idx = 1:size(folders,1)
        f = "react-control"
        cart = false
        if (contains(folders[idx], "CART") || contains(folders[idx], "cart"))
            cart = true
            f = "cartesianSolver_test"
        end
        stats = createStatsDict()
        println(folders[idx])
        for exper in expers
            if !isfile("/home/rozliv/iCub/"* f * "/data/"*f_name*folders[idx]*"/"*string(exper)*"/param.log") && !cart
                continue
            end
            mArm, mArm_data, sArm, sArm_data, data, general_data, param_idx, param_idx2, start_time, end_time = parseData("/home/rozliv/iCub/"* f * "/data/"*f_name*folders[idx]*"/"*string(exper)*"/", exper, posThr, oriThr, dT, cart)
            obstacle_data = nothing
            if isfile("/home/rozliv/iCub/"* f * "/data/"*f_name*folders[idx]*"/"*string(exper)*"_obs/data.log")
                obstacle_data = CSV.read("/home/rozliv/iCub/"* f * "/data/"*f_name*folders[idx]*"/"*string(exper)*"_obs/data.log", DataFrame; header=0) |> Matrix{Float64}
                obstacle_data = obstacle_data[(obstacle_data[:,3] .>= start_time) .& (obstacle_data[:,3] .<= end_time), :]
                obst_time = obstacle_data[:,3] .- start_time
                obst_dist = vecnorm_row(obstacle_data[:,4:6].-data[1:size(obstacle_data,1),mArm.x_t])
                obst_dist_ref = vecnorm_row(obstacle_data[:,4:6].-data[1:size(obstacle_data,1),mArm.x_d])
                # if show_plots
                    p = plot(; title="Obst dist")
                    plot!(obst_time, obst_dist; label="EE vs obstacle dist", linewidth=3);
                    plot!(obst_time, obst_dist_ref; label="Target vs obstacle dist", linewidth=3)
                    display(p)
                # end
            end
            if (show_plots)
                allPlots(general_data, mArm, mArm_data, data, param_idx, exper-1, cart, folders[idx])
            end
            if exper == 3
                stats["cart_vel_profile"] = movmean(mArm_data["cart_vels_norm"], 20)    
                stats["joint_vel_profile"] = movmean(mArm_data["joint_vels_norm"],20)
            end
            push!(stats["rmse_refpos"], sqrt(mean(abs2,mArm_data["refpos_error"])))
            push!(stats["rmse_pos"], sqrt(mean(abs2,mArm_data["pos_error"])))
            push!(stats["rmse_ori"], sqrt(mean(abs2,mArm_data["theta"])))
            push!(stats["rmse_refori"], sqrt(mean(abs2,mArm_data["reftheta"])))
            push!(stats["cart_acc_max"], maximum(filter(!isnan,mArm_data["cart_acc_norm"])))
            push!(stats["cart_acc_sum"], sum(filter(!isnan,mArm_data["cart_acc_norm"])))
            push!(stats["cart_acc_mean"], sum(filter(!isnan,mArm_data["cart_acc_norm"]))/size(filter(!isnan,mArm_data["cart_acc_norm"]),1))
            push!(stats["cart_jerk_max"], maximum(filter(!isnan, mArm_data["cart_jerk_norm"])))
            push!(stats["cart_jerk_mean"], sum(filter(!isnan,mArm_data["cart_jerk_norm"]))/size(filter(!isnan,mArm_data["cart_jerk_norm"]),1))
            push!(stats["cart_jerk_sum"], sum(filter(!isnan,mArm_data["cart_jerk_norm"])))
            push!(stats["cart_jerk_median"], median(filter(!isnan,mArm_data["cart_jerk_norm"])))
            push!(stats["dists"], mArm_data["distance"])
            push!(stats["joint_jerk_max"], maximum(filter(!isnan, mArm_data["joint_jerk_norm"])))
            push!(stats["joint_jerk_sum"], sum(filter(!isnan,mArm_data["joint_jerk_norm"])))
            push!(stats["joint_jerk_median"], median(filter(!isnan,mArm_data["joint_jerk_norm"])))
            push!(stats["joint_jerk_mean"], sum(filter(!isnan,mArm_data["joint_jerk_norm"]))/size(filter(!isnan,mArm_data["joint_jerk_norm"]),1))
            push!(stats["max_ttr"], maximum(mArm_data["time_to_reach"]))
            push!(stats["med_ttr"], median(mArm_data["time_to_reach"]))
            push!(stats["mean_ttr"], mean(mArm_data["time_to_reach"]))
            push!(stats["meanpos_ttr"], mean(mArm_data["refpos_error"][mArm_data["target_lastindex"]]))
            push!(stats["meanori_ttr"], mean(mArm_data["reftheta"][mArm_data["target_lastindex"]]))
            push!(stats["stdpos_ttr"], std(mArm_data["refpos_error"][mArm_data["target_lastindex"]]))
            push!(stats["stdori_ttr"], std(mArm_data["reftheta"][mArm_data["target_lastindex"]]))
            push!(stats["num_ttr"], sum((mArm_data["refpos_error"][mArm_data["ttr_idx"]] .< posThr) .& (mArm_data["reftheta"][mArm_data["ttr_idx"]] .< oriThr)) / length(mArm_data["ttr_idx"]))
            push!(stats["time_ttr"], mArm_data["time_to_reach"])
            push!(stats["time_pos_ttr"], mArm_data["time_to_reach_pos"])
            push!(stats["time_ori_ttr"], mArm_data["time_to_reach_ori"])
            push!(stats["pos_ttr"], mArm_data["refpos_error"][mArm_data["target_lastindex"]])
            push!(stats["pos_ttr2"], mArm_data["refpos_error"][mArm_data["ttr_idx"]])
            push!(stats["ori_ttr"], mArm_data["reftheta"][mArm_data["target_lastindex"]])
            push!(stats["ori_ttr2"], mArm_data["reftheta"][mArm_data["ttr_idx"]])
            push!(stats["reached_targets"], (mArm_data["refpos_error"][mArm_data["ttr_idx"]] .< posThr) .& (mArm_data["reftheta"][mArm_data["ttr_idx"]] .< oriThr))
            push!(stats["reached_pos"], mArm_data["refpos_error"][mArm_data["ttr_idx"]] .< posThr)
            push!(stats["reached_ori"], mArm_data["reftheta"][mArm_data["ttr_idx"]] .< oriThr)
            push!(stats["cart_jerk_norm"], mArm_data["cart_jerk_norm"])
            push!(stats["joint_jerk_norm"], mArm_data["joint_jerk_norm"])
            push!(stats["cart_acc_norm"], mArm_data["cart_acc_norm"])
            push!(stats["refpos_error"], mArm_data["refpos_error"])
            push!(stats["pos_error"], mArm_data["pos_error"])
            push!(stats["ori_error"], mArm_data["theta"])
            push!(stats["refori_error"], mArm_data["reftheta"])
            if show_plots
                # scatter3d(data[mArm_data["target_lastindex"],mArm.x_d[1]], data[mArm_data["target_lastindex"],mArm.x_d[2]], data[mArm_data["target_lastindex"],mArm.x_d[3]]; markersize=5)
                # scatter3d!(data[mArm_data["target_lastindex"][stats["reached_targets"][end]],mArm.x_d[1]], data[mArm_data["target_lastindex"][stats["reached_targets"][end]],mArm.x_d[2]], data[mArm_data["target_lastindex"][stats["reached_targets"][end]],mArm.x_d[3]]; markersize=10)
                # display(plot3d!(; camera=(0,0)))
            end
            if (exper < 4)
                println(sum((mArm_data["refpos_error"][mArm_data["ttr_idx"]] .< posThr) .& (mArm_data["reftheta"][mArm_data["ttr_idx"]] .< oriThr)), " ", length(mArm_data["ttr_idx"]))
            end
        end
        multistats[folders[idx]] = stats;
    end
    return multistats
end


function allPlots(general_data, mArm, mArm_data, data, param_idx, exper, cart=false, name="")
    names = ["1st torso", "2nd torso", "3rd torso", "1st shoulder", "2nd shoulder", "3rd shoulder", "1st elbow", "2nd elbow", "1st wrist", "2nd wrist"];
    skip2 = collect(1:10)
    if cart
        lims = [-20 -10 -50 -95 0 -37 15 -60 -80 -20; 70 10 50 10 160 80 106 60 25 25]
    else
        lims = transpose(hcat([values(param_idx.blim)...], [values(param_idx.ulim)...]))
    end
    if cart
        vel_lims_x = nothing
        vel_lims_b = nothing
        vel_lims_u = nothing
    else
        vel_lims_x = general_data.t
        vel_lims_b = data[:,mArm.vLimAdapted[2*skip2.-1]]
        vel_lims_u = data[:,mArm.vLimAdapted[2*skip2]]
    end
    plotJoints(general_data.t, mArm_data["joint_vels"][:,skip2], names[skip2], "Joint vel [deg/s]", name, "Joint velocities for Exp " * string(exper), vel_lims_x, vel_lims_b, vel_lims_x, vel_lims_u)
    # plotJoints(general_data.t[2:end], mArm_data["joint_vels2"][:,skip2], names[skip2], "Joint vel [deg/s]", name, "Joint velocities for Exp " * string(exper), vel_lims_x, vel_lims_b, vel_lims_x, vel_lims_u)
    plotJoints(general_data.t, data[:,mArm.qIntegrated[skip2]], names[skip2], "Joint pos [deg]", name, "Joint positions for Exp " * string(exper),  [0,general_data.t[end]], repeat(lims[1:1, skip2], outer=2), [0,general_data.t[end]], repeat(lims[2:2, skip2], outer=2))
    # plotJoints(general_data.t, data[:, mArm.q[skip2]], names[skip2], "Joint pos [deg]", name, "Joint positions for Exp " * string(exper),  [0,general_data.t[end]], repeat(lims[1:1, skip2], outer=2), [0,general_data.t[end]], repeat(lims[2:2, skip2], outer=2))
    plotTargetErrors(general_data.t, data, mArm, mArm_data, exper, name)
    if (exper > 1)
        plotTargets(general_data.t, data, mArm, exper, name)
    end
end


function plotJerks(multistats, baseline, expers, patterns=[""])
    for pat in patterns
        plot_array = [] 
        for key in ["joint_jerk_max", "joint_jerk_median", "joint_jerk_mean", "cart_jerk_max", "cart_jerk_median", "cart_jerk_mean"] #"rmse_pos", "rmse_ori", "rmse_pose", 
            # println(key)
            offset = 0
            baseline_vals = []
            for (stat, values) in pairs(multistats)
                if stat == baseline
                    baseline_vals = values
                end
            end
            p1 = plot(; xticks = (1:4, expers))
            for (stat, values) in pairs(multistats)
                if contains(stat, pat)
                    vals = values[key]
                    if !isempty(baseline_vals)
                        vals = vals ./ baseline_vals[key]
                    end
                    lab = stat # split(stat, '_')[1]
                    plot!(vals; linewidth = 4, markershape = :circle, markersize= 6, title = key, legend = key == "cart_jerk_mean", label=lab)
                    offset += 0.0
                    # println("\t\t" * stat * ":  " * string(values[key]))
                end
            end
            push!(plot_array, p1)
        end
        pp = plot(plot_array...; size=(1000,1000))
        display(pp)
        savefig(pp, "jerks_summ.png")

    end
end

function plotVelProfile(multistats)
    plot_array = [] 
    ylabels = ["Velocity [deg/s]", "Velocity [m/s]"]
    for key in ["joint_vel_profile", "cart_vel_profile"] 
    #    for pat in ["MJ", "None"]
            offset = 0
            p1 = plot()
            for (stat, values) in pairs(multistats)
                # if contains(stat, pat) || contains(stat, "cart")
                    vals = values[key][950:1650]
                    lab = stat # split(stat, '_')[1]
                    plot!((1:length(vals)) .+ offset,vals; ylabel=ylabels[1+Int(key == "cart_vel_profile")], linewidth = 4, title = key, legend = key == "cart_vel_profile", label=lab)
                    offset +=  60
                # end
            end
            push!(plot_array, p1)
        # end
    end
    title = scatter(ones(3), marker=0,markeralpha=0, annotations=(2, 1, text("Exp 3", font(18))), titlefontsize=18,axis=([], false), grid=false, leg=false,size=(200,100))
    pp = plot(plot_array...;layout=grid(2,1), size=(1000,1000))
    p = plot(title, pp,layout=grid(2,1,heights=[0.02,0.98]))
    display(p)
    savefig(p, "vel_profile2.png")
end

function plotJerksInTime(multistats, step, baseline, suptitle)
    labels = []
    max_joint_jerks = Vector{Vector{Float64}}()
    max_cart_jerks = Vector{Vector{Float64}}()
    baseline_joint = nothing
    baseline_cart = nothing
    for (stat, values) in pairs(multistats)
        idx = 1
        lab = stat # split(stat, '_')[1]
        push!(labels, lab)
        vals_joint = Vector{Float64}()
        vals_cart = Vector{Float64}()
        while idx < length(values["joint_jerk_norm"][1])-step
            push!(vals_joint, maximum(values["joint_jerk_norm"][1][idx:idx+step]))
            push!(vals_cart, maximum(values["cart_jerk_norm"][1][idx:idx+step]))
            idx = idx+step
        end
        push!(vals_joint, maximum(values["joint_jerk_norm"][1][idx:end]))
        push!(vals_cart, maximum(values["cart_jerk_norm"][1][idx:end]))
        if stat == baseline
            baseline_joint = deepcopy(vals_joint)
            baseline_cart = deepcopy(vals_cart)
        end
        push!(max_joint_jerks, vals_joint)
        push!(max_cart_jerks, vals_cart)
    end
    p1 = plot(;title="Joint jerk norm")
    for i = 1:length(labels)
        plot!((1:length(max_cart_jerks[i])).*t, max_joint_jerks[i]./maximum(baseline_joint); label=labels[i], lw=3)
    end
    
    p2 = plot(;title="Cart jerk norm")
    for i = 1:length(labels)
        plot!((1:length(max_cart_jerks[i])).*t, max_cart_jerks[i]./maximum(baseline_cart); label=labels[i], lw=3)
    end
    title = scatter(ones(3), marker=0,markeralpha=0, annotations=(2, 1, text(suptitle, font(18))), titlefontsize=18,axis=([], false), grid=false, leg=false,size=(200,100))
    p = plot(title, p1, p2,layout=grid(3,1,heights=[0.02,0.49, 0.49]))
    # display(plot(p1,p2; layout=grid(2,1), size=(800,600)))
    display(p)
    savefig(p, "jerks_" * replace(suptitle, " " => "_") * ".png")
end

function plotNextPosError(multistats, step, suptitle)
    labels = []
    max_pos_error = Vector{Vector{Float64}}()
    max_ori_error = Vector{Vector{Float64}}()
    mean_pos_error = Vector{Vector{Float64}}()
    mean_ori_error = Vector{Vector{Float64}}()
    for (stat, values) in pairs(multistats)
        idx = 1
        lab = stat # split(stat, '_')[1]
        push!(labels, lab)
        pos_err = Vector{Float64}()
        ori_err = Vector{Float64}()
        pos_err_mean = Vector{Float64}()
        ori_err_mean = Vector{Float64}()
        while idx < length(values["pos_error"][1])-step
            push!(pos_err, maximum(values["pos_error"][1][idx:idx+step]))
            push!(ori_err, maximum(values["ori_error"][1][idx:idx+step]))
            push!(pos_err_mean, mean(values["pos_error"][1][idx:idx+step]))
            push!(ori_err_mean, mean(values["ori_error"][1][idx:idx+step]))
            idx = idx+step
        end
        push!(pos_err, maximum(values["pos_error"][1][idx:end]))
        push!(ori_err, maximum(values["ori_error"][1][idx:end]))
        push!(pos_err_mean, mean(values["pos_error"][1][idx:end]))
        push!(ori_err_mean, mean(values["ori_error"][1][idx:end]))
        push!(max_pos_error, pos_err)
        push!(max_ori_error, ori_err)
        push!(mean_pos_error, pos_err_mean)
        push!(mean_ori_error, ori_err_mean)
    end
    p1 = plot(;title="Next Pos error", lw=3)
    for i = 1:length(labels)
        plot!((1:length(max_pos_error[i])).*t, max_pos_error[i]; lw=3, label=labels[i])
        # plot!((1:length(mean_pos_error[i])).*t, mean_pos_error[i]; label=labels[i])
    end
    
    p2 = plot(;title="Next ori error", lw=3)
    for i = 1:length(labels)
        plot!((1:length(max_ori_error[i])).*t, max_ori_error[i];  lw=3, label=labels[i])
        # plot!((1:length(mean_ori_error[i])).*t, mean_ori_error[i]; label=labels[i])
    end
    title = scatter(ones(3), marker=0,markeralpha=0, annotations=(2, 1, text(suptitle, font(18))), titlefontsize=18,axis=([], false), grid=false, leg=false,size=(200,100))
    p = plot(title, p1, p2,layout=grid(3,1,heights=[0.02,0.49, 0.49]))
    display(p)
    savefig(p, "nextpos_" * replace(suptitle, " " => "_") * ".png")

    # display(plot(p1,p2; layout=grid(2,1), size=(800,600)))
end


function plotStreamedBoxplots(multistats, suptitle)
    labels = []
    refpos = Vector{Vector{Float64}}()
    refori = Vector{Vector{Float64}}()
    pos = Vector{Vector{Float64}}()
    ori = Vector{Vector{Float64}}()
    for (stat, values) in pairs(multistats)
        lab = stat # split(stat, '_')[1]
        push!(labels, lab)
        push!(refpos, values["refpos_error"][1])
        push!(refori, values["refori_error"][1])
        push!(pos, values["pos_error"][1])
        push!(ori, values["ori_error"][1])
    end
    p1 = plot(;title="Next Pos error", xrotation = 90)
    for i = 1:length(labels)
        boxplot!([labels[i]], pos[i]; legend=:false)
    end
    p2 = plot(;title="Next Ori error", xrotation = 90)
    for i = 1:length(labels)
        boxplot!([labels[i]], ori[i]; legend=:false)
    end
    p3 = plot(;title="Target Pos error", xrotation = 90)
    for i = 1:length(labels)
        boxplot!([labels[i]], refpos[i]; legend=:false)
    end
    p4 = plot(;title="Target Ori error", xrotation = 90)
    for i = 1:length(labels)
        boxplot!([labels[i]], refori[i]; legend=:false)
    end
    title = scatter(ones(3), marker=0,markeralpha=0, annotations=(2, 1, text(suptitle, font(18))), titlefontsize=18,axis=([], false), grid=false, leg=false,size=(200,100))
    p5 = plot(p1,p2,p3,p4; layout=grid(2,2), size=(800,600))
    p = plot(title, p5,layout=grid(2,1,heights=[0.02,0.98]))
    display(p)
    savefig(p, "streamed_" * replace(suptitle, " " => "_") * ".png")

    # display()

end

function plotTTRBoxplots(multistats, suptitle)
    labels = Vector{String}()
    reached_num = Vector{Vector{Float64}}()
    time_ttr_all = Vector{Vector{Float64}}()
    pos_ttr_all = Vector{Vector{Float64}}()
    ori_ttr_all = Vector{Vector{Float64}}()
    time_ttr = Vector{Vector{Float64}}()
    time_pos_ttr = Vector{Vector{Float64}}()
    time_ori_ttr = Vector{Vector{Float64}}()
    pos_ttr = Vector{Vector{Float64}}()
    ori_ttr = Vector{Vector{Float64}}()
    reached_targets = Vector{Vector{Float64}}()
    unreached_targets = Vector{Vector{Float64}}()
    for (stat, values) in pairs(multistats)
        lab = stat # split(stat, '_')[1]
        push!(labels, lab)
        # push!(time_ttr_all, values["time_ttr"][1])
        push!(reached_num, [sum(values["reached_pos"][1]),sum(values["reached_ori"][1]), sum(values["reached_targets"][1])])
        push!(pos_ttr_all, values["pos_ttr2"][1])
        push!(ori_ttr_all, values["ori_ttr2"][1])
        push!(time_ttr, values["time_ttr"][1][values["reached_targets"][1]])
        push!(time_pos_ttr, values["time_pos_ttr"][1][values["reached_pos"][1]])
        push!(time_ori_ttr, values["time_ori_ttr"][1][values["reached_ori"][1]])
        push!(pos_ttr, values["pos_ttr2"][1][values["reached_targets"][1]])
        push!(ori_ttr, values["ori_ttr2"][1][values["reached_targets"][1]])
        indexes = collect(1:length(values["time_ttr"][1]))
        push!(reached_targets, indexes[values["reached_targets"][1]])
        push!(unreached_targets, indexes[.!values["reached_targets"][1]])
    end
    p1 = plot(;title="Reached targets", xrotation = 90)
    labs = ["Pos reached", "Ori reached", "Pos+Ori reached"]    
    ctg = repeat(labs, inner = length(labels))
    nam = repeat(labels, outer = 3)
    reached_num = Array(reduce(hcat,reached_num)')
    
     # Redefine unique for `CategoricalArray` types to return a categorical array, rather than a regular vector/array. 
     @eval function Base.unique(ctg::CategoricalArray) # can be run in REPL instead
        l = levels(ctg)
        newctg = CategoricalArray(l)
        levels!(newctg, l)
    end
    
    nam = categorical(nam; levels = labels)
    ctg = categorical(ctg; levels = labs)

    groupedbar!(nam, reached_num, group = ctg, color = [:red :green :blue], legend=:bottomright)
    # for i = 1:length(labels)
    #     # boxplot!([labels[i]], time_ttr_all[i]; legend=:false)
    #     println(reached_num[i])
    #     groupedbar!([labels[i]], reached_num[i]; legend=:false)
    # end
    p2 = plot(;title="Pos error", legend=:false, yaxis=:log)
    for i = 1:length(labels)
        boxplot!([labels[i]], pos_ttr_all[i])
    end
    p3 = plot(;title="Ori error",legend=:false, yaxis=:log)
    for i = 1:length(labels)
        boxplot!([labels[i]], ori_ttr_all[i])
    end
    p4 = plot(;title="Time to reach (Pos reached)")
    for i = 1:length(labels)
        boxplot!([labels[i]], time_pos_ttr[i]; legend=:false)
    end
    p5 = plot(;title="Time to reach (Ori reached)")
    for i = 1:length(labels)
        boxplot!([labels[i]], time_ori_ttr[i]; legend=:false)
    end
    p4 = plot(;title="Time to reach")
    for i = 1:length(labels)
        boxplot!([labels[i]], time_ttr[i]; legend=:false)
    end
    # p5 = plot(;title="Pos error (reached only)")
    # for i = 1:length(labels)
    #     boxplot!([labels[i]], pos_ttr[i]; legend=:false)
    # end
    # p6 = plot(;title="Ori error (reached only)")
    # for i = 1:length(labels)
    #     boxplot!([labels[i]], ori_ttr[i]; legend=:false)
    # end
    # p7 = plot(;title="Reached targets statistics", ylim=(1-0.5*length(labels) - 0.2, 0.7), yticks = (1 .- 0.5.*collect(1:length(labels)), labels))
    # scatter!(reached_targets[1], ones(size(reached_targets[1])) .- 0.5; mc=:green, msc=:green, markersize=3, label="Reached", legend=:false)
    # scatter!(unreached_targets[1], ones(size(unreached_targets[1])).- 0.5; mc=:red, msc=:red, markersize=3, label="Not reached")   
    # for i = 2:length(labels)
    #     scatter!(reached_targets[i], ones(size(reached_targets[i])) .- 0.5*i; mc=:green, msc=:green, markersize=3, legend=:false)
    #     scatter!(unreached_targets[i], ones(size(unreached_targets[i])).- 0.5*i; mc=:red, msc=:red, markersize=3, legend=:false)   
    # end
    # l = @layout [grid(1,3),
    #             grid(1,3),
    #                 grid(1,1)]
    p8 = plot(p1,p2,p3,p4; layout=grid(1,4), size=(1200,500), xrotation=20)
    title = scatter(ones(3), marker=0,markeralpha=0, annotations=(2, 1, text(suptitle, font(18))), titlefontsize=18,axis=([], false), grid=false, leg=false,size=(200,100))
    p = plot(title, p8,layout=grid(2,1,heights=[0.02,0.98]))
    display(p)
    savefig(p, "target_reach_" * replace(suptitle, " " => "_") * ".png")
end


posThr = 0.006
oriThr = deg2rad(5)
dT = 0.02
t = 0.5
st = Int(trunc(t/dT))
obsRes = true
showFigs = false
if obsRes
    f_name = "tes"
    folders = ["lims_cn", "lims_mn", "lims_cod", "lims_mod"] #, "mag_03_mn", "mag_03_cn", "mag_03_mod", "mag_03_cod", "mag_04_cod", "mag_05_mn", "mag_01_mn"] #, "react_o1_meannormal", "react_o01_meannormal", "react_o01_maxnormal"]
    # folders = ["mag_03_mn", "mag_03_cn", "mag_03_mod", "mag_03_cod", "mag_04_cod"] #, "react_o1_meannormal", "react_o01_meannormal", "react_o01_maxnormal"]
    # folders = ["lims_cn", "lims_mn", "lims_cod", "lims_mod", "mag_03_cn", "mag_03_mn"]
    folders = ["lims2_cod5", "con3_cod4", "lims2_cod4", "con3_cod4_virtual"]#, "NEO_MJ", "NEO_None"]
    # folders = ["R_33"]
    for i in [1,3,4]
        stats_exp1 = analyzeData(f_name, folders, [i], posThr, oriThr, dT, showFigs)
        plotNextPosError(stats_exp1, st, "Exp "*string(i-1))
        plotJerksInTime(stats_exp1, st, folders[1], "Exp "*string(i-1))
        plotStreamedBoxplots(stats_exp1, "Exp "*string(i-1))
        if i == 3
            plotTTRBoxplots(stats_exp1, "Exp 2")
        end
    end
    # stats_exp1345 = analyzeData(f_name, folders, [1,3,4], posThr, oriThr, dT, false)
    # plotJerks(stats_exp1345, folders[1], ["Exp 0", "Exp 2", "Exp 3", "Exp 4"])
else
    f_name = "test"
    folders = ["reactv7_30_None",  "NEO-NM_30_None", "ipopt_30_None", "cart"] #"results/NEO_30_None",
    folders = ["cart"]
    # folders = ["reactv7_30_MinJerk", "NEO-NM_30_MinJerk", "ipopt_30_MinJerk", "cart"] #"results/NEO_30_None",
    # folders = ["react_MJ", "NEO_MJ", "Nguyen_MJ", "cart"]
    # folders = ["react_None", "NEO_None", "Nguyen_None", "cart"]
    folders = ["react_MJ", "NEO_MJ", "Nguyen_MJ", "react_None", "NEO_None", "Nguyen_None", "cart"]
    # stats_exp2 = analyzeData(f_name, folders, [2], posThr, oriThr, dT, true)

    folders = ["react_normal", "react_manip2", "react_manip3"]#, "react_INF10sqrt", "react_INF10"]#, "react_INF"]
    stats_exp3 = analyzeData(f_name, folders, [3], posThr, oriThr, dT, showFigs)
    # folders = ["react_MJ", "NEO_MJ", "Nguyen_MJ", "react_None", "NEO_None", "Nguyen_None"]
    stats_exp4 = analyzeData(f_name, folders, [4], posThr, oriThr, dT, showFigs)
    stats_exp5 = analyzeData(f_name, folders, [5], posThr, oriThr, dT, showFigs)
    stats_exp45 = analyzeData(f_name, folders, [3,4,5], posThr, oriThr, dT, true)

    plotVelProfile(stats_exp3)
    # plotTTRBoxplots(stats_exp2, "Exp 1")
    plotTTRBoxplots(stats_exp3, "Exp 2")
    plotNextPosError(stats_exp4, st, "Exp 3")
    plotNextPosError(stats_exp5, st, "Exp 4")
    plotJerksInTime(stats_exp4, st, folders[1], "Exp 3")
    plotJerksInTime(stats_exp5, st, folders[1], "Exp 4")
    plotStreamedBoxplots(stats_exp4, "Exp 3")
    plotStreamedBoxplots(stats_exp5, "Exp 4")
    plotJerks(stats_exp45, folders[1], ["Exp 1", "Exp 2", "Exp 3", "Exp 4"])
end