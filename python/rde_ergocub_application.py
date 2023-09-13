# Author Ines Sorrentino - 2023

import h5py
import numpy as np
import manifpy
from progressbar import ProgressBar
import pickle
import time

import bipedal_locomotion_framework.bindings as blf
import idyntree.bindings as idyn
import icub_models

import matplotlib.pyplot as plt

def populate_numerical_data(file_object):
    data = {}

    for key, value in file_object.items():
        if not isinstance(value, h5py._hl.group.Group):
            continue
        if key == "#refs#":
            continue
        if key == "log":
            continue
        if "data" in value.keys():
            data[key] = {}
            data[key]["data"] = np.squeeze(np.array(value["data"]))
            data[key]["timestamps"] = np.squeeze(np.array(value["timestamps"]))

            # In yarp telemetry v0.4.0 the elements_names was saved.
            if "elements_names" in value.keys():
                elements_names_ref = value["elements_names"]
                data[key]["elements_names"] = [
                    "".join(chr(c[0]) for c in value[ref])
                    for ref in elements_names_ref[0]
                ]
        else:
            if key == "robot_logger_device":
                data = populate_numerical_data(file_object=value)
            else:
                data[key] = populate_numerical_data(file_object=value)

    return data

def build_kin_dyn(robot_name, param_handler, robot_model_path):
    if (robot_model_path == ""):
        robot_model_path = icub_models.get_model_file(robot_name)

    joint_list = param_handler.get_parameter_vector_string("joint_list")
    ft_list = param_handler.get_group("FT").get_parameter_vector_string("associated_joints")

    ml = idyn.ModelLoader()
    ml.loadReducedModelFromFile(str(robot_model_path), joint_list + ft_list)

    # Create kindyn for full model
    kindyn = idyn.KinDynComputations()
    kindyn.loadRobotModel(ml.model())
    ok = kindyn.setFrameVelocityRepresentation(idyn.BODY_FIXED_REPRESENTATION)
    if (not ok):
        raise ValueError("BODY_FIXED_REPRESENTATION not set")

    # create submodel list
    sub_model_creator = blf.robot_dynamics_estimator.SubModelCreator()
    sub_model_creator.setModelAndSensors(ml.model(), ml.sensors())
    sub_model_creator.setKinDyn(kindyn)
    sub_model_creator.createSubModels(param_handler)

    sub_model_list = sub_model_creator.getSubModelList()

    kindyn_wrapper_list = []
    for idx in range(len(sub_model_list)):
        kindyn_sm = blf.robot_dynamics_estimator.KinDynWrapper()
        if (not kindyn_sm.setModel(sub_model_list[idx])):
            raise ValueError("kindyn_sm.initialize")
        kindyn_wrapper_list.append(kindyn_sm)

    idynEstimator = idyn.ExtWrenchesAndJointTorquesEstimator()
    idynEstimator.setModelAndSensors(ml.model(), ml.sensors())

    return kindyn, sub_model_list, kindyn_wrapper_list, idynEstimator

def build_initial_state(data, handler, idynEstimator, kindyn):
    x0 = blf.robot_dynamics_estimator.RobotDynamicsEstimatorOutput()

    joint_list_config = handler.get_parameter_vector_string("joint_list")
    n_joints = len(joint_list_config)
    joint_list_dataset = data['joints_state']['positions']['elements_names']

    joints_idx_in_dataset = []

    for i in range(n_joints):
        idx = [n for n, x in enumerate(joint_list_dataset) if joint_list_config[i] in x]
        joints_idx_in_dataset.append(idx)

    x0.ds = data['joints_state']['velocities']['data'][0, joints_idx_in_dataset]

    s = data['joints_state']['positions']['data'][0, joints_idx_in_dataset]
    dds = data['joints_state']['accelerations']['data'][0, joints_idx_in_dataset]

    gear_ratio = (100.0, -160.0, 100.0, 100.0, 100.0, 160.0)
    torque_constant = (0.111, 0.047, 0.047, 0.111, 0.111, 0.025)
    r_ktau  = [a * b for a, b in zip(gear_ratio, torque_constant)]

    i_m = data['motors_state']['currents']['data'][0, joints_idx_in_dataset]

    x0.tau_m = [a * b for a, b in zip(r_ktau, i_m)]

    x0.tau_F = np.zeros(x0.tau_m.shape)


    # Set contact information and specify the unknown wrench on the base link
    base_link = handler.get_parameter_string("base_link")
    fixedFrameIdx = idynEstimator.model().getFrameIndex(base_link)
    fullBodyUnknowns = idyn.LinkUnknownWrenchContacts(idynEstimator.model())
    fullBodyUnknowns.clear()
    fullBodyUnknowns.addNewUnknownFullWrenchInFrameOrigin(idynEstimator.model(), fixedFrameIdx)

    # Initialize variables for estimation
    expectedFT = idyn.SensorsMeasurements(idynEstimator.sensors())
    estimatedContacts = idyn.LinkContactWrenches(idynEstimator.model())
    estimatedTau = idyn.JointDOFsDoubleArray(idynEstimator.model())

    # TO BE CHANGED IN FLOATING BASE CASA
    gravity = idyn.Vector3()
    gravity.zero()
    gravity[2] = -blf.math.StandardAccelerationOfGravitation

    s_idyn = idyn.JointPosDoubleArray(n_joints)
    ds_idyn = idyn.JointDOFsDoubleArray(n_joints)
    dds_idyn = idyn.JointDOFsDoubleArray(n_joints)

    for j in range(n_joints):
        s_idyn.setVal(j, s[j].item())
        ds_idyn.setVal(j, x0.ds[j].item())
        dds_idyn.setVal(j, dds[j].item())

    idynEstimator.updateKinematicsFromFixedBase(s_idyn, ds_idyn, dds_idyn, fixedFrameIdx, gravity)
    idynEstimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns, expectedFT, estimatedContacts, estimatedTau)

    ft_associated_joints = handler.get_group("FT").get_parameter_vector_string("associated_joints")
    ft_list = handler.get_group("FT").get_parameter_vector_string("names")

    ftFromModel = dict()
    ft_offsets = dict()
    temp1 = dict()
    temp2 = dict()
    ftWrench = idyn.Wrench()
    for ft in ft_list:
        ftIndex = idynEstimator.sensors().getSensorIndex(idyn.SIX_AXIS_FORCE_TORQUE, ft)
        assert expectedFT.getMeasurement(idyn.SIX_AXIS_FORCE_TORQUE, ftIndex, ftWrench)
        ftFromModel[ft] = ftWrench.toNumPy()

        ft_temp = data['FTs'][ft]['data'][0]

        ft_offsets[ft] = ft_temp - ftFromModel[ft]
        temp1[ft] = ftFromModel[ft]

    x0.ftWrenches = temp1

    kindyn.setFrameVelocityRepresentation(idyn.BODY_FIXED_REPRESENTATION)

    H_base: np.ndarray = np.eye(4)
    H_base_idyn = idyn.Transform().Identity()
    pos = idyn.Position().FromPython(H_base[:3, 3])
    R = idyn.Rotation().FromPython(H_base[:3, :3])

    H_base_idyn.setPosition(pos)
    H_base_idyn.setRotation(R)

    v_base_idyn = idyn.Twist()
    v_base_idyn = v_base_idyn.FromPython(np.zeros(6))

    s_kindyn = idyn.VectorDynSize(n_joints)
    ds_kindyn = idyn.VectorDynSize(n_joints)
    dds_kindyn = idyn.VectorDynSize(n_joints)

    s_kindyn = s_kindyn.FromPython(s.reshape(n_joints))
    ds_kindyn = ds_kindyn.FromPython(x0.ds.reshape(n_joints))

    ok = kindyn.setRobotState(H_base_idyn, s_kindyn,
                              v_base_idyn, ds_kindyn,
                              gravity)
    if not ok:
        raise RuntimeError("Failed to set the robot state")

    base_acc = idyn.Vector6()
    base_acc.zero()

    acc_list = handler.get_group("ACCELEROMETER").get_parameter_vector_string("names")
    acc_frames = handler.get_group("ACCELEROMETER").get_parameter_vector_string("frames")
    temp3 = dict()
    i = 0
    for acc in acc_list:
        frame_acceleration = kindyn.getFrameAcc(acc_frames[i], base_acc, dds_idyn)
        H = kindyn.getWorldTransform(acc_frames[i])
        acc_R_world = H.getRotation().inverse()
        velocity = kindyn.getFrameVel(acc_frames[i]).toNumPy()
        acc_meas = np.zeros(3)
        i = i + 1
    #x0.accelerometerBiases = temp3

    gyro_list = handler.get_group("GYROSCOPE").get_parameter_vector_string("names")
    gyro_frames = handler.get_group("GYROSCOPE").get_parameter_vector_string("frames")
    temp4 = dict()
    i = 0
    for gyro in gyro_list:
        velocity = kindyn.getFrameVel(gyro_frames[i]).toNumPy()
        vel_meas = np.zeros(3)
        i = i + 1
    # x0.gyroscopeBiases = temp4

    contact_list = handler.get_group("EXTERNAL_CONTACT").get_parameter_vector_string("frames")
    temp5 = dict()
    for contact in contact_list:
        temp5[contact] = np.zeros(6)
    x0.contactWrenches = temp5

    return x0, ft_offsets, joints_idx_in_dataset

def build_estimator_input(data, index, handler, ft_offsets, joints_idx_in_dataset):
    input_rde = blf.robot_dynamics_estimator.RobotDynamicsEstimatorInput()

    input_rde.basePose = manifpy.SE3(position=[0.0, 0.0, 0.0], quaternion=[0.0, 0, 0, 1.0])
    input_rde.baseVelocity = manifpy.SE3Tangent([0.0]*6)
    input_rde.baseAcceleration = manifpy.SE3Tangent([0.0]*6)

    input_rde.jointPositions = data['joints_state']['positions']['data'][index, joints_idx_in_dataset]
    input_rde.jointVelocities = data['joints_state']['velocities']['data'][index, joints_idx_in_dataset]

    input_rde.motorCurrents = data['motors_state']['currents']['data'][index, joints_idx_in_dataset]

    ft_list = handler.get_group("FT").get_parameter_vector_string("names")
    temp1 = dict()
    for ft in ft_list:
        ft_temp = data['FTs'][ft]['data'][index]
        temp1[ft] = ft_temp - ft_offsets[ft]
    input_rde.ftWrenches = temp1

    acc_list = handler.get_group("ACCELEROMETER").get_parameter_vector_string("names")
    temp2 = dict()
    for acc in acc_list:
        temp2[acc] = np.zeros(3)
    input_rde.linearAccelerations = temp2

    gyro_list = handler.get_group("GYROSCOPE").get_parameter_vector_string("names")
    temp3 = dict()
    for gyro in gyro_list:
        temp3[gyro] = np.zeros(3)
    input_rde.angularVelocities = temp3

    return input_rde

def parse_output(output, handler):
    rde = dict()

    rde['ds'] = []
    rde['tau_m'] = []
    rde['tau_F'] = []
    rde['tau_j'] = []
    rde['ft'] = dict()
    rde['ft_bias'] = dict()
    rde['acc_bias'] = dict()
    rde['gyro_bias'] = dict()
    rde['contact'] = dict()

    ft_list = handler.get_group("FT").get_parameter_vector_string("names")
    acc_list = handler.get_group("ACCELEROMETER").get_parameter_vector_string("names")
    gyro_list = handler.get_group("GYROSCOPE").get_parameter_vector_string("names")
    contact_list = handler.get_group("EXTERNAL_CONTACT").get_parameter_vector_string("frames")

    for ft in ft_list:
        rde['ft'][ft] = []

    for contact in contact_list:
        rde['contact'][contact] = []

    for index in range(len(output)):
        rde['ds'].append(output[index].ds)
        rde['tau_m'].append(output[index].tau_m)

        n_joints = len(rde['ds'][0])
        
        rde['tau_F'].append(output[index].tau_F)

        rde['tau_j'].append(output[index].tau_m - output[index].tau_F)

        for ft in ft_list:
            rde['ft'][ft].append(output[index].ftWrenches[ft])
            
        for contact in contact_list:
            rde['contact'][contact].append(output[index].contactWrenches[contact])

    rde['ds'] = np.array(rde['ds'])
    rde['tau_m'] = np.array(rde['tau_m'])
    rde['tau_F'] = np.array(rde['tau_F'])
    rde['tau_j'] = np.array(rde['tau_j'])
    for ft in ft_list:
        rde['ft'][ft] = np.array(rde['ft'][ft])
    for contact in contact_list:
        rde['contact'][contact] = np.array(rde['contact'][contact])

    return rde

def main():
    robot = 'ergoCubSN000_r_leg'
    param_handler = blf.parameters_handler.YarpParametersHandler()
    assert param_handler.set_from_filename("config/"+robot+"/config.ini")

    robot_name = param_handler.get_group("GENERAL").get_parameter_string("robot_name")
    try:
        robot_model_path = param_handler.get_group("GENERAL").get_parameter_string("urdf_path")
    except:
        robot_model_path = ""

    # Create sub-models related objects
    [kindyn, sub_model_list, kindyn_wrapper_list, idynEstimator] = build_kin_dyn(robot_name, param_handler.get_group("MODEL"), robot_model_path)

    # Crete estimator
    estimator = blf.robot_dynamics_estimator.RobotDynamicsEstimator.build(param_handler,
                                                                          kindyn,
                                                                          sub_model_list,
                                                                          kindyn_wrapper_list)

    # Load dataset
    dataset = param_handler.get_group("GENERAL").get_parameter_string("dataset")
    f = h5py.File(dataset, 'r')
    data = populate_numerical_data(f)

    # Build and set initial state
    [initial_state, ft_offset, joints_idx_in_dataset] = build_initial_state(data, param_handler.get_group("MODEL"), idynEstimator, kindyn)
    ok = estimator.setInitialState(initial_state)
    if not ok:
        raise ValueError("Failed setting initial state")

    output_rde = []

    last_index = data['joints_state']['positions']['data'].shape[0]

    pbar = ProgressBar()
    for index in pbar(range(int(last_index))):
        index1 = index
        start_time = time.time()
        input_rde = build_estimator_input(data, index1, param_handler.get_group("MODEL"), ft_offset, joints_idx_in_dataset)
        ok = estimator.set_input(input_rde)
        if not ok:
            raise ValueError("Failed setting the input")

        ok = estimator.advance()
        if not ok:
            raise ValueError("Failed advancing")

        output_rde.append(estimator.get_output())

    rde = parse_output(output_rde, param_handler.get_group("MODEL"))

    timestr = time.strftime("%Y%m%d_%H%M%S")
    with open('output_estimation/' + robot + '/workspace_' + timestr + '.pickle', 'wb') as handle:
        pickle.dump([rde, data], handle)

if __name__ == "__main__":
    main()
