
from ira.ira.arm_movements import AbstractArmMovements
from facial_recog import FacialRecog
from camera import Camera
from individual_interaction import IndivInteraction
# For how the robot behaves when it is not 'locked' onto one person,
# for the time inbetween when it is searching for a new person to paint or just doing it's own thing.

# This level can also deal with how we react to a person disappearing before they
# have been painted, for example.  So all the possible OUTCOMES from an individual interaction.
# each face/face_encoding can have a STATUS, or a list of past outcomes:
# e.g. known since this date, painted x times.
# maybe a dict for each face:
# { 
# "interactions": {"no" : 1, "date": "march 3 1908", "outcome": "left before being painted."}
# }

# I wonder if in future there could be a way to associate voices with faces, maybe by taking
# lots of images and assigning a face encoding to a voice encoding or a microphone number...
# or if in an exhibit, this could be done in a little test booth before entry or something.

# Do find_faces, check_faces, then for each one check if close, known, centred

# Take all the faces and pick one based on info above: pick the cloest unknown, then furthest unknown, then a known...
# Take that face an use to create an individual interaction session -> to get robot to move to centre the face.
# Speaking to and painting that face is operated from the individual interaction. Also eye movements.  ROS? threading? asyncio?
# Result of indiv interaction is fedd back to this class... e.g. finished the painting successfully, person disappeared, etc.
# create a dataset_interactions.dat file to save info for each face encoding and all past interactions with it.  

arm_movement = AbstractArmMovements()
face_recog = FacialRecog()
cam = Camera()

found_faces = [] # look at the latest info from facial_recog node 
# if num_faces = 0, keep looking
while len(found_faces) == 0:
    arm_movement.scan()
    _, image = cam.read()
    # TODO add in some other nice random robot/eye movements for whilst scanning.
    # TODO add in random talking: 1) little time has past: just some random thoughts
    # 2) lots of time has past: I'm lonely!
    # All 3 of these happen in parallel and must be easy to stop relatively quickly 
    found_faces = face_recog.find_faces(image)

# if num_faces = 1, then start IndivInteration with this face.
# (when with a larger crowd, probably won't happen, just keep looking until you find someone who is close enough!
# So you don't choose someone far away and then try get them to come closer.
if len(found_faces) == 1:
    # Just start individial interaction session with this one face.
    indiv_interaction = IndivInteraction(found_faces[0])
# if num_faces > 1 AND largest_face = True, then start IndivInteration with this face.
else:
    # Decide which face to use.
    use_idx = -1
    max_known_face_size = 0
    max_unknown_face_size = 0
    unknown_face = False
    for num, face in enumerate(found_faces):
        if face.known == False and face.size > max_unknown_face_size:
            max_unknown_face_size = face.size
            use_idx = num
            unknown_face = True
        if unknown_face == False:
            if face.size > max_known_face_size:
                max_known_face_size = face.size
                use_idx = num
    indiv_interaction = IndivInteraction(found_faces[use_idx])

outcome = indiv_interaction.run()
    





