import struct

def unpack(fmt,buf):
    return struct.unpack_from('<' + (fmt * int(len(buf)/struct.calcsize(fmt))),buf)

class OriginalModel(object):
    def __init__(self,**kwargs):
        self.id = kwargs.get('id')
        self.grasping_rescale = kwargs.get('grasping_rescale')
        self.tags = kwargs.get('tags')
        self.maker = kwargs.get('maker')
        self.model = kwargs.get('model')
        self.source = kwargs.get('source')
        self.description = kwargs.get('description')
        self.barcode = kwargs.get('barcode')
        self.acquisition_method_name = kwargs.get('acquisition_method_name')
        self.concave_filled = kwargs.get('concave_filled')
        self.rotationally_symmetric = kwargs.get('rotationally_symmetric')
        self.inner_concavity = kwargs.get('inner_concavity')
        self.recognition_id = kwargs.get('recognition_id')

class ScaledModel(object):
    def __init__(self,**kwargs):
        self.id = kwargs.get('id')
        self.scale = kwargs.get('scale')
        self.original_model_id = kwargs.get('original_model_id',kwargs.get('model_id'))


class Mesh(object):
    def __init__(self,**kwargs):
        self.original_model_id = kwargs.get('original_model_id',kwargs.get('model_id'))
        self.vertices = kwargs.get('vertices',kwargs.get('vertex_list'))
        self.triangles = kwargs.get('triangles',kwargs.get('triangle_list'))

class ModelSet(object):
    def __init__(self,**kwargs):
        self.name = kwargs.get('name')
        self.original_model_ids = kwargs.get('original_model_ids',kwargs.get('model_ids'))
        
class Grasp(object):
    def __init__(self,**kwargs):
        self.id = kwargs.get('id')
        self.scaled_model_id = kwargs.get('scaled_model_id',kwargs.get('model_id'))
        self.pregrasp_joints = kwargs.get('pregrasp_joints')
        self.grasp_joints = kwargs.get('grasp_joints')
        self.energy = kwargs.get('energy')
        self.pregrasp_pose = kwargs.get('pregrasp_pose')
        self.grasp_pose = kwargs.get('grasp_pose')
        self.source_id = kwargs.get('source_id')
        self.pregrasp_clearance = kwargs.get('pregrasp_clearance')
        self.cluster_rep = kwargs.get('cluster_rep')
        self.hand_name = kwargs.get('hand_name')
        self.table_clearance = kwargs.get('table_clearance')
        self.compliant_copy = kwargs.get('compliant_copy')
        self.compliant_original_id = kwargs.get('compliant_original_id')
        self.scaled_quality = kwargs.get('scaled_quality')
        self.fingertip_object_collision = kwargs.get('fingertip_object_collision')

class Hand(object):
    def __init__(self,**kwargs):
        self.name = kwargs.get('name')
        self.path = kwargs.get('relative_path',kwargs.get('path'))

class DB(object):
    _original_models = None
    _scaled_models = None
    _meshes = None
    _model_sets = None
    _grasps = None
    _hands = None
    
    @staticmethod
    def original_models():
        return DB._original_models.values()
    
    @staticmethod
    def original_model(id):
        if hasattr(id,'original_model_id'):
            id = id.original_model_id
        if DB._original_models.has_key(id):
            return DB._original_models[id]
        raise KeyError('No model with id %d' % id)
    
    @staticmethod
    def scaled_models():
        return DB._scaled_models.values()
    
    @staticmethod
    def scaled_model(id):
        if hasattr(id,'scaled_model_id'):
            id = id.scaled_model_id
        if DB._scaled_models.has_key(id):
            return DB._scaled_models[id]
        raise KeyError('No model with id %d' % id)
    
    @staticmethod
    def scaled_models_for(original_model):
        if hasattr(original_model,'original_model_id'):
            id = original_model.original_model_id
        else:
            id = original_model
        models = []
        for scaled_model in DB.scaled_models():
            if scaled_model.original_model_id == id:
                scaled_model.append(models)
        return models
    
    @staticmethod
    def meshes():
        return DB._meshes.values()
    
    @staticmethod
    def mesh(model):
        if hasattr(model,'original_model_id'):
            id = model.original_model_id
        elif isinstance(model,OriginalModel):
            id = model.id
        else:
            id = model
        if DB._meshes.has_key(id):
            return DB._meshes[id]
        raise KeyError('No mesh for id %d' % id)
    
    @staticmethod
    def model_sets():
        return DB._model_sets.values()
    
    @staticmethod
    def model_set(name):
        if DB._model_sets.has_key(name):
            return DB._model_sets[name]
        raise KeyError('No model set with name %s' % name)
    
    @staticmethod
    def grasps():
        return DB._grasps.values()
    
    @staticmethod
    def grasp(key):
        if isinstance(key,ScaledModel):
            id = key.id
            for grasp in DB.grasps():
                if grasp.scaled_model_id == id:
                    return grasp
        elif isinstance(key,OriginalModel):
            scaled_models = DB.scaled_models_for(key)
            if len(scaled_models) > 1:
                raise KeyError('Original model with id %d has more than one scaled model!' % key.id)
            return DB.grasp(scaled_models[0])
        else:
            if DB._grasps.has_key(key):
                return DB._grasps[key]
        raise KeyError('No grasp for id %d' % key)
    
    @staticmethod
    def hands():
        return DB._hands.values()
    
    @staticmethod
    def hand(name):
        if DB._hands.has_key(name):
            return DB._hands[name]
        raise KeyError('No hand with name %s' % name)

    @staticmethod
    def load():
        import psycopg2
    
        conn = psycopg2.connect(host='localhost', database='household_objects',
                                user='willow', password='willow')
        cur = conn.cursor()
        
        try:
            DB._load_original_models(cur)
            DB._load_scaled_models(cur)
            DB._load_meshes(cur)
            DB._load_model_sets(cur)
            DB._load_grasps(cur)
            DB._load_hands(cur)
        finally:
            conn.close()
        
    @staticmethod
    def _load_original_models(cur):
        DB._orignal_models = {}
        
        table_name = 'original_model'
        
        cur.execute('SELECT * FROM %s' % table_name)
        for record in cur:
            args = {}
            for col, value in zip(cur.description,record):
                name = col.name
                if name.startswith(table_name):
                    name = name[len(table_name)+1:]
                args[name] = value
            model = OriginalModel(**args)
            DB._orignal_models[model.name] = model
    
    @staticmethod
    def _load_scaled_models(cur):
        DB._scaled_models = {}
        
        table_name = 'scaled_model'
        cur.execute('SELECT * FROM %s' % table_name)
        for record in cur:
            args = {}
            for col, value in zip(cur.description,record):
                name = col.name
                if name.startswith(table_name):
                    name = name[len(table_name)+1:]
                args[name] = value
            model = ScaledModel(**args)
            DB._scaled_models[model.name] = model
    
    @staticmethod
    def _load_meshes(cur):
        DB._meshes = {}
        
        table_name = 'scaled_model'
        cur.execute('SELECT * FROM %s' % table_name)
        for record in cur:
            args = {}
            for col, value in zip(cur.description,record):
                name = col.name
                if name.startswith(table_name):
                    name = name[len(table_name)+1:]
                if name == 'vertex_list':
                    value = unpack('d',value)
                elif name == 'triangle_list':
                    value = unpack('i',value)
                args[name] = value
            mesh = Mesh(**args)
            DB._meshes[mesh.original_model_id] = mesh
    
    @staticmethod
    def _load_model_sets(cur):
        DB._scaled_models = {}
        
        table_name = 'model_set'
        cur.execute('SELECT * FROM %s' % table_name)
        for record in cur:
            args = {}
            for col, value in zip(cur.description,record):
                name = col.name
                if name.startswith(table_name):
                    name = name[len(table_name)+1:]
                args[name] = value
            model_set = ModelSet(**args)
            DB._model_sets[model_set.name] = model_set
    
    @staticmethod
    def _load_grasps(cur):
        DB._scaled_models = {}
        
        table_name = 'grasp'
        cur.execute('SELECT * FROM %s' % table_name)
        for record in cur:
            args = {}
            for col, value in zip(cur.description,record):
                name = col.name
                if name.startswith(table_name):
                    name = name[len(table_name)+1:]
                args[name] = value
            grasp = Grasp(**args)
            DB._grasps[grasp.id] = grasp
    
    @staticmethod
    def _load_hands(cur):
        DB._scaled_models = {}
        
        table_name = 'hand'
        cur.execute('SELECT * FROM %s' % table_name)
        for record in cur:
            args = {}
            for col, value in zip(cur.description,record):
                name = col.name
                if name.startswith(table_name):
                    name = name[len(table_name)+1:]
                args[name] = value
            hand = Hand(**args)
            DB._hand[hand.name] = hand
        