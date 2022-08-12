using Plots
using CSV
using DataFrames
using LinearAlgebra
using Rotations
using Statistics
using NaNStatistics
using OrderedCollections
using StatsBase

gr(size=(800,800), legend=true, markerstrokewidth=0,markersize=2)


function prepareData(d)
    solverExitCode = 2+3
    timeToSolveProblem_s = 3+3
    t = d[:,3].-d[1,3]
    time_diff = diff(t)
    replace!(x -> x >0.5 ? NaN : x, time_diff)
    time_to_reach = []
    indexes_reach = []
    start = t[1]
    for i = 2:size(time_diff,1)
        if isequal(time_diff[i],NaN) && !isequal(time_diff[i-1], NaN)
            push!(time_to_reach, t[i]-start)
            push!(indexes_reach, i)
        end
        if isequal(time_diff[i-1], NaN) && !isequal(time_diff[i], NaN)
            start = t[i]
        end
    end
    return (t=t, time_diff=time_diff, indexes_reach=indexes_reach, time_to_reach=time_to_reach, solver_time=d[:,timeToSolveProblem_s], solver_exit=d[:,solverExitCode])
end

vecnorm_col(A) = [norm(@view A[:,i]) for i=1:size(A,2)]
vecnorm_row(A) = [norm(@view A[i,:]) for i=1:size(A,1)]

function prepareArmData(d, indexes, dT)
    data = Dict()
    data["pos_error"] = vecnorm_row(d[:, indexes.x_n]-d[:, indexes.x_t])
    data["refpos_error"] = vecnorm_row(d[:, indexes.x_d]-d[:, indexes.x_t])
    data["pose_diff"] = zeros(size(d)[1],1)
    data["theta"] = zeros(size(d)[1],1)
    for i = 1:size(d)[1]
        ori_des = RotMatrix(AngleAxis(d[i, indexes.o_n[end]], d[i, indexes.o_n[1]], d[i, indexes.o_n[2]], d[i, indexes.o_n[3]]))
        ori_cur = RotMatrix(AngleAxis(d[i, indexes.o_t[end]], d[i, indexes.o_t[1]], d[i, indexes.o_t[2]], d[i, indexes.o_t[3]]))
        Hr = Matrix{Float64}(I,4,4)
        He = Matrix{Float64}(I,4,4)
        Hr[1:3,1:3] = ori_des
        He[1:3,1:3] = ori_cur
        Hr[1:3,4]=d[i, indexes.x_n]
        He[1:3,4]=d[i, indexes.x_t]
        H = ori_des * transpose(ori_cur)
        data["theta"][i] = rotation_angle(H)
        data["pose_diff"][i] = norm(Hr-He)
    end
    data["cart_vels"] = diff(d[:,indexes.x_t], dims=1)./dT    
    data["cart_vels_norm"] = vecnorm_row(data["cart_vels"])
    data["cart_acc"] = diff(data["cart_vels"], dims=1)./dT    
    data["cart_acc_norm"] = vecnorm_row(data["cart_acc"])
    data["cart_jerk"] = diff(data["cart_acc"], dims=1)./dT
    data["cart_jerk_norm"] = vecnorm_row(data["cart_jerk"])
    data["joint_vels"] = d[:,indexes.qDot]
    data["joint_vels2"] = diff(d[:,indexes.qIntegrated], dims=1)./dT
    data["joint_vels3"] = diff(d[:,indexes.q], dims=1)./dT
    data["joint_vels_norm"] = vecnorm_row(d[:,indexes.qDot])
    data["joint_vels_norm2"] = vecnorm_row(data["joint_vels2"])
    data["joint_vels_norm3"] = vecnorm_row(data["joint_vels3"])
    
    data["joint_jerk"] = diff(diff(d[2:end,indexes.qDot], dims=1)./dT, dims=1)./dT
    data["joint_jerk2"] = diff(diff(data["joint_vels2"], dims=1)./dT, dims=1)./dT
    data["joint_jerk3"] = diff(diff(data["joint_vels3"], dims=1)./dT, dims=1)./dT
    data["joint_jerk_norm"] = vecnorm_row(data["joint_jerk"]) 
    data["joint_jerk_norm2"] = vecnorm_row(data["joint_jerk2"])
    data["joint_jerk_norm3"] = vecnorm_row(data["joint_jerk3"])
    data["distance"] = sum(abs.(diff(d[:,indexes.qIntegrated], dims=1)))
    return data
end


function parseData(folder)

    params = CSV.read(folder*"param.log", DataFrame; header=0)

    param_idx = (part=params[1,1], chainDof=params[1,2], selCol=params[1,3], blim=params[1,4:2:4+params[1,2]*2-1], ulim=params[1,5:2:5+params[1,2]*2-1], vmax=params[1,4+params[1,2]*2], trajSpeed=params[1,5+params[1,2]*2], tol=params[1,6+params[1,2]*2], globalTol=params[1,7+params[1,2]*2], 
                dT=params[1,8+params[1,2]*2], timeLimit=params[1,9+params[1,2]*2], stiff=params[1,10+params[1,2]*2], colPoints=params[1,(11:13).+params[1,2]*2], hitt=params[1,14+params[1,2]*2], restPos=params[1,15+params[1,2]*2], oriControl=params[1,16+params[1,2]*2], 
                gaze=params[1,17+params[1,2]*2])
    print(param_idx.globalTol)
    param_idx2 = nothing
    if (size(params,2) > 38)
        param_idx2 = (part=params[1,39], chainDof=params[1,40], selCol=params[1,41], blim=params[1,42:2:42+params[1,40]*2-1], ulim=params[1,43:2:43+params[1,40]*2-1])
    end

    df = CSV.read(folder*"data.log", DataFrame; header=0) |> Matrix{Float64}
    df = df[1:300,:]
    data = df[:,4:end]

    mArm = (chainDOF=1, avoidance=4, x_d=5:7, x_t=8:10, x_n=11:13, o_d=14:17, o_t=18:21, o_n=22:25, qDot=26:35, q=36:45, qIntegrated=46:55, vLimAdapted=56:75);
    general_data = prepareData(df)
    mArm_data = prepareArmData(data, mArm, param_idx.dT)

    if (size(data)[2] > 75)
        sArm = (chainDOF=76, avoidance=77, x_d=78:80, x_t=81:83, x_n=84:86, o_d=87:90, o_t=91:94, o_n=95:98, qDot=99:108, q=109:118, qIntegrated=119:128, vLimAdapted=129:148);
        sArm_data = prepareArmData(data, sArm, param_idx.dT)
    else
        sArm = nothing
        sArm_data = nothing
    end
    return mArm, mArm_data, sArm, sArm_data, data, general_data, param_idx, param_idx2
end


function plotJoints(t, data, names, ylab, xline1=nothing, yline1=nothing, xline2=nothing, yline2=nothing)
    p1=scatter(t[100:end], data[100:end,:];
    xlabel = "Time [s]",
    ylabel = ylab,
    layout = grid(3,3),
    linewidth = 3,
    title = ["$n" for j=1:1, n in names],
    legend = false,
    size=(800,800)
    )
    if !isnothing(xline1)
        # plot!(xline1, yline1; linewidth=2)
        # plot!(xline2, yline2; linewidth=2) 
    end
    display(p1)
end

function createStatsDict()
    stats = Dict()
    stats["rmse_refpos"] = Vector{Float64}()
    stats["rmse_pos"] = Vector{Float64}()
    stats["rmse_ori"] = Vector{Float64}()
    stats["rmse_pose"] = Vector{Float64}()
    stats["cart_jerk_max"] = Vector{Float64}()
    stats["cart_jerk_sum"] = Vector{Float64}()
    stats["mean_time"] = Vector{Float64}()
    stats["dists"] = Vector{Float64}()
    stats["total_time"] = Vector{Float64}()
    stats["max_ttr"] = Vector{Float64}()
    stats["med_ttr"] = Vector{Float64}()
    stats["mean_ttr"] = Vector{Float64}()
    stats["meanpos_ttr"] = Vector{Float64}()
    stats["meanori_ttr"] = Vector{Float64}()
    stats["stdpos_ttr"] = Vector{Float64}()
    stats["stdori_ttr"] = Vector{Float64}()
    stats["num_ttr"] = Vector{Float64}()
    stats["acc_norm_sum"] = Vector{Float64}()
    return stats
end


function allPlots(general_data, mArm, mArm_data, data, param_idx)
    names = ["1st torso", "2nd torso", "3rd torso", "1st shoulder", "2nd shoulder", "3rd shoulder", "1st elbow", "2nd elbow", "1st wrist", "2nd wrist"];
    skip2 = [[1];collect(3:10)]
    lims = transpose(hcat([values(param_idx.blim)...], [values(param_idx.ulim)...]))

    plotJoints(general_data.t, mArm_data["joint_vels"][:,skip2], names[skip2], "Joint vel [deg/s]", general_data.t, data[:,mArm.vLimAdapted[2*skip2.-1]], general_data.t, data[:,mArm.vLimAdapted[2*skip2]])
    plotJoints(general_data.t[2:end], mArm_data["joint_vels2"][:,skip2], names[skip2], "Joint vel [deg/s]",general_data.t, data[:,mArm.vLimAdapted[2*skip2.-1]], general_data.t, data[:,mArm.vLimAdapted[skip2*2]])
    plotJoints(general_data.t[2:end], mArm_data["joint_vels3"][:,skip2], names[skip2], "Joint vel [deg/s]",general_data.t, data[:,mArm.vLimAdapted[2*skip2.-1]], general_data.t, data[:,mArm.vLimAdapted[skip2*2]])
    plotJoints(general_data.t, data[:,mArm.qIntegrated[skip2]], names[skip2], "Joint pos [deg]", [0,general_data.t[end]], repeat(lims[1:1, skip2], outer=2), [0,general_data.t[end]], repeat(lims[2:2, skip2], outer=2))
    plotJoints(general_data.t, data[:, mArm.q[skip2]], names[skip2], "Joint pos [deg]",  [0,general_data.t[end]], repeat(lims[1:1, skip2], outer=2), [0,general_data.t[end]], repeat(lims[2:2, skip2], outer=2))

    p1 = scatter(general_data.t[2:end], general_data.time_diff; xlabel = "Time (s)", ylabel = "Taken Time (s)", label = "Time diff")
    scatter!(general_data.t,general_data.solver_time; label="Solver time")
    plot!([general_data.t[1], general_data.t[end]],[param_idx.dT, param_idx.dT]; series_type= :path, label="Period", linewidth=3)

    p2 = scatter(general_data.t,general_data.solver_exit; xlabel = "time (s)", ylabel = "Exit code [-]");

    p3 = scatter(general_data.t,abs.(mArm_data["theta"]); label="Orientation error")
    scatter!(general_data.t,mArm_data["pos_error"]; label="Position error")
    scatter!(general_data.t,mArm_data["pose_diff"]; label="Pose error")

    p4 = plot(general_data.t[2:end], mArm_data["cart_vels_norm"]) #scatter(general_data.t[2:end], sum(abs2.(diff(deg2rad(mArm_data["joint_vels"]), dims=1)), dims=2))
    # display(plot(p1, p2, p3, p4; layout=grid(4,1), size=(800,800)))

    p1 = scatter(general_data.t, data[:,mArm.x_d[1]]; label="Target X", ylabel="Position [m]")
    scatter!(general_data.t, data[:,mArm.x_n[1]]; label="Next X")
    scatter!(general_data.t, data[:,mArm.x_t[1]]; label="Current X")

    p2 = scatter(general_data.t, data[:,mArm.x_d[2]]; label="Target Y", ylabel="Position [m]")
    scatter!(general_data.t, data[:,mArm.x_n[2]]; label="Next Y")
    scatter!(general_data.t, data[:,mArm.x_t[2]]; label="Current Y")

    p3 = scatter(general_data.t, data[:,mArm.x_d[3]]; label="Target Z", ylabel="Position [m]")
    scatter!(general_data.t, data[:,mArm.x_n[3]]; label="Next Z")
    scatter!(general_data.t, data[:,mArm.x_t[3]]; label="Current Z")
    p4 = scatter(general_data.t, mArm_data["pos_error"]; label="Position error", ylabel="Position [m]")
    display(plot(p1, p2, p3, p4; layout=grid(4,1), size=(800,800), title="Target position"))
    
    xs = data[:,mArm.x_d[1]]
    ys = data[:,mArm.x_d[2]]
    zs = data[:,mArm.x_d[3]]
    # condition data for nearly isometric 3D plot 
    x12, y12, z12 = extrema(xs), extrema(ys), extrema(zs)
    d = maximum([diff([x12...]),diff([y12...]),diff([z12...])])[1] / 2
    xm, ym, zm = mean(x12),  mean(y12),  mean(z12) 

    # plot data
    p4 = Plots.plot(; xlabel="x",ylabel="y",zlabel="z", aspect_ratio=:equal, grid=:true, size=(800,800), title="3D position")
    # plot!(xlims=(xm-d,xm+d), ylims=(ym-d,ym+d), zlims=(zm-d,zm+d))
    plot!(;camera=(90,0))    #(azimuth,elevation) ???
    scatter!(xs, ys, zs, label="Target pos")
    scatter!(data[:,mArm.x_n[1]], data[:,mArm.x_n[2]], data[:,mArm.x_n[3]]; label="Next Pos")
    # display(scatter!(data[:,mArm.x_t[1]], data[:,mArm.x_t[2]], data[:,mArm.x_t[3]]; label="Current Pos"))
    # if exper == 1
    #     time_offset = 10.    
    #     index = findfirst(general_data.t .> time_offset)

    #     dists = mArm_data["refpos_error"][index:end]
    #     dists2 = mArm_data["pos_error"][index:end]
    #     println(size(dists))
    #     plot(; xlabel="x",ylabel="y", aspect_ratio=:equal, grid=:true, size=(800,800), title="Reference tracking")
    #     display(plot(general_data.t[index:end].-general_data.t[index], dists))
    #     display(plot(general_data.t[index:end].-general_data.t[index], dists2))
    # elseif exper == 5
    #     time_offset = 3.
    #     index = findfirst(general_data.t .> time_offset)
    #     dists = mArm_data["refpos_error"][index:end-50]
    #     dists2 = mArm_data["pos_error"][index:end-50]
    #     println(size(dists))
    #     plot(; xlabel="x",ylabel="y", aspect_ratio=:equal, grid=:true, size=(800,800), title="Reference tracking")
    #     display(plot(general_data.t[index:end-50].-general_data.t[index], dists))   
    #     display(plot(general_data.t[index:end-50].-general_data.t[index], dists2))   
    # elseif exper == 3
    #     plot(; xlabel="x",ylabel="y", aspect_ratio=:equal, grid=:true, size=(800,800), title="Reference tracking")
    #     display(plot(general_data.t, mArm_data["refpos_error"]))
    # end
end


function plotGraphs(multistats, patterns=[""])
    for pat in patterns
        plot_array = [] 
        for key in ["rmse_pos", "rmse_ori", "rmse_pose", "acc_norm_sum", "cart_jerk_max", "cart_jerk_sum"]
            # println(key)
            offset = 0
            p1 = plot()
            for (stat, values) in pairs(multistats)
                if contains(stat, pat)
                    plot!((1:2:2*size(values[key],1)) .+ offset, values[key]; linewidth = 4, markershape = :circle, markersize= 6, title = key, legend = key == "rmse_pose", label=stat)
                    offset += 0.2
                    # println("\t\t" * stat * ":  " * string(values[key]))
                end
            end
            push!(plot_array, p1)
        end
        display(plot(plot_array...; size=(1000,1000)))
    end

    for pat in patterns
        plot_array = [] 
        for key in ["med_ttr", "mean_ttr", "max_ttr", "num_ttr", "meanpos_ttr", "meanori_ttr", "stdpos_ttr", "stdori_ttr"]
            offset = 0
            p1 = plot()
            for (stat, values) in pairs(multistats)
                if contains(stat, pat)
                    plot!((1:2:2*size(values[key],1)) .+ offset, values[key]; linewidth = 4, markershape = :circle, markersize= 6, title = key, legend = key == "num_ttr", label=stat)
                    offset += 0.2
                end
            end
            push!(plot_array, p1)
        end
        c = false
        for (stat, values) in pairs(multistats)
            if contains(stat, pat) && haskey(values, "vel_profile")
                c = true
                break
            end
        end
        l = @layout grid(2,4)
        if c
            p1 = plot()
            offset = 0
            for (stat, values) in pairs(multistats)
                if contains(stat, pat) && haskey(values, "vel_profile")
                    plot!(range(0+offset,stop=1, length=(size(values["vel_profile"],1))), values["vel_profile"]; linewidth = 3, title = "vel_profile", legend = false, label=stat)
                    offset += 0.02
                end
            end
            push!(plot_array, p1)
            l = @layout [grid(2,4)
                        b]
        end
        display(plot(plot_array...; layout= l, size=(1000,1000)))
    end
end


function analyzeData(f_name, folders, show_plots=false)
    multistats = OrderedDict()
    # 95 percent CI je 1.96* std/N
    
    for idx = 1:size(folders,1)
        stats = createStatsDict()
        println(folders[idx])
        for exper in [1,2,3,4,5]
            if !isfile("/home/rozliv/iCub/react-control/data/"*f_name*folders[idx]*"_"*string(exper)*"/param.log")
                continue
            end
            mArm, mArm_data, sArm, sArm_data, data, general_data, param_idx, param_idx2 = parseData("/home/rozliv/iCub/react-control/data/"*f_name*folders[idx]*"_"*string(exper)*"/")
            
            if (show_plots)
                allPlots(general_data, mArm, mArm_data, data, param_idx)
            end

            # if exper == 1
            #     time_offset = 10.    
            #     index = findfirst(general_data.t .> time_offset) 
            #     offset = 249
            #     println(findall(vecnorm_row(data[index:end, mArm.x_d] .- data[index:index, mArm.x_d]) .< 0.0001))
            # elseif exper == 5
            #     time_offset = 10.    
            #     index = findfirst(general_data.t .> time_offset)
            #     offset = 488
            #     println(findall(vecnorm_row(data[index:end, mArm.x_d] .- data[index:index, mArm.x_d]) .< 0.000001))
            # end
            if exper == 3
                stats["vel_profile"] = movmean(mArm_data["cart_vels_norm"], 20)    
            end
            if exper == 3 || exper == 2
                push!(stats["med_ttr"], median(general_data.time_to_reach))    
                push!(stats["mean_ttr"], mean(general_data.time_to_reach))
                push!(stats["max_ttr"], maximum(general_data.time_to_reach))
                push!(stats["num_ttr"], size(general_data.time_to_reach,1))
                push!(stats["meanpos_ttr"], mean(mArm_data["pos_error"][general_data.indexes_reach]))    
                push!(stats["meanori_ttr"], mean(mArm_data["theta"][general_data.indexes_reach]))   
                push!(stats["stdpos_ttr"], std(mArm_data["pos_error"][general_data.indexes_reach]))
                push!(stats["stdori_ttr"], std(mArm_data["theta"][general_data.indexes_reach]))   
            end
            push!(stats["rmse_refpos"], sqrt(mean(abs2,mArm_data["refpos_error"])))
            push!(stats["rmse_pos"], sqrt(mean(abs2,mArm_data["pos_error"])))
            push!(stats["rmse_ori"], sqrt(mean(abs2,mArm_data["theta"])))
            push!(stats["rmse_pose"], sqrt(mean(abs2,mArm_data["pose_diff"])))
            push!(stats["cart_jerk_max"], maximum(mArm_data["cart_jerk_norm"]))
            push!(stats["cart_jerk_sum"], sum(mArm_data["cart_jerk_norm"])/size(mArm_data["cart_jerk_norm"],1))
            push!(stats["mean_time"], mean(filter(!isnan,general_data.time_diff)))
            push!(stats["dists"], mArm_data["distance"])
            push!(stats["total_time"], general_data.t[end])
            push!(stats["acc_norm_sum"], sum(mArm_data["cart_acc_norm"])/size(mArm_data["cart_acc_norm"],1))
        end
        multistats[folders[idx]] = stats;
    end
    return multistats
end



f_name = "preliminary5/"
folders = ["r0_p1_o1", "r0_p1_o5", "r0_p5_o1", "r0_p5_o5", "r0_p5_o1", "r0_p5_o5", "r0_p10_o1", "r0_p10_o5",
"r0.01_p1_o1", "r0.01_p1_o5", "r0.01_p5_o1", "r0.01_p5_o5", "r0.01_p5_o1", "r0.01_p5_o5", "r0.01_p10_o1", "r0.01_p10_o5",
"r1_p1_o1", "r1_p1_o5", "r1_p5_o1", "r1_p5_o5", "r1_p5_o1", "r1_p5_o5", "r1_p10_o1", "r1_p10_o5"] #, "o2", "o5", "o10"]
folders = ["o1"]
# folders = ["m0_r0.01_p0.5_o1", "m0_r0.01_p1_o1", "m0_r0.01_p5_o1",
# "m0_r0.01_p0.5_o0.5", "m0_r0.01_p1_o0.5", "m0_r0.01_p5_o0.5",
# "m0_r1_p0.5_o1", "m0_r1_p1_o1", "m0_r1_p5_o1",
# "m0_r1_p0.5_o0.5", "m0_r1_p1_o0.5", "m0_r1_p5_o0.5"]
multistats = analyzeData(f_name, folders, true)

# plotGraphs(multistats, ["","r0_", "r1", "r0.01", "p1_", "p5", "p10", "o1", "o5"]) # ["r1", "r0.01", "r0_", "p5", "p1", "p0.5", "m0.03", "m0_", "m1"])

# TODO: add repeatability u lemniscate, p2p, circular casti