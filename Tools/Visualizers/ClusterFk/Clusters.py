import networkx as nx

from bitstring import BitArray
class Worker:

    def __init__(self, comp_id, bonus, isManager, skill_num):
        self.company = comp_id
        self.bonus = bonus
        self.is_manager = isManager
        self.skill_num = skill_num
        self.skills = BitArray()
        self.rawSkills = []


    def init_skills(self, skill_num, current_skill_list):
        for skill in current_skill_list:
            if skill in self.rawSkills:
                self.skills.set(True, current_skill_list[skill])


    def add_raw_skill(self, skill):
        self.rawSkills.append(skill)

    def edge_val(self, co_worker):
        if self.is_manager:
            workPotential = 0
        else:
            common_bits = log_and(self.skills, co_worker.skills)
            workPotential = count_bits(common_bits)*count_bits(common_bits.invert())

        if self.company == co_worker.company:
            bonusPotential = self.bonus * co_worker.bonus
        else:
            bonusPotential = 0
        return (workPotential + bonusPotential)

    def __str__(self):
        return '%s | %s | %s | %s\n' % (self.company, 'Manager' if self.is_manager else 'Developer', self.skill_num, ','.join(self.rawSkills))


def count_bits(bit_array):
  num = 0
  for bit in bit_array:
    if bit == 1:
      num += 1
  return num


def log_and(bits1, bits2):
    if len(bits1) == len(bits2):
        return bits1 & bits2
    if len(bits1) > len(bits2):
        maxBits = bits1
        minBits = bits2
    elif len(bits2) > len(bits1):
        maxBits = bits2
        minBits = bits1
    minBits <<= (maxBits - minBits)
    minBits >>= (maxBits - minBits)
    return maxBits & minBits




def read_input():
    with open('input/a_solar.txt', 'r') as file:
        status = 0
        arguments = {}
        for line in file:
            status = process_line(line, status, arguments)


def process_line(line, status, arguments):
    if status == 0:
        # Layout size
        line_data = line.split(' ')
        w = int(line_data[0])
        h = int(line_data[1])
        arguments.update({
            'W': w,
            'H': h,
            'currentRow': 0,
            'prevRow': '#' * w
        })
        new_status = 1
    elif status == 1:
        # Layout
        prev_row = arguments['prevRow']
        prev_char = '#'
        for i in range(len(line)):
            char = line[i]
            if char == '_' or char == 'M':
                node_name = '%s:%s:%s' % (char, i, arguments['currentRow'])
                layout.add_node(node_name, x=i, y=arguments['currentRow'], m=char == 'M')
                if prev_char == '_' or prev_char == 'M':
                    prev_node_name = '%s:%s:%s' % (prev_char, i - 1, arguments['currentRow'])
                    layout.add_edge(node_name, prev_node_name)
                top_char = prev_row[i]
                if top_char == '_' or top_char == 'M':
                    top_node_name = '%s:%s:%s' % (top_char, i, arguments['currentRow'] - 1)
                    layout.add_edge(node_name, top_node_name)
            prev_char = char
        arguments['prevRow'] = line
        arguments['currentRow'] += 1
        if arguments['currentRow'] >= arguments['H']:
            new_status = 2
        else:
            new_status = 1
    elif status == 2:
        # Developer Count
        arguments.update({
            'devCount': int(line),
            'currentDevs': 0
        })
        new_status = 3
    elif status == 3:
        # Developer
        d_comp, d_bonus, d_skillcount, *d_skills = line.split(' ')
        if d_comp not in companyMap:
            companyMap[d_comp] = companyMap['___ai']
            companyMap['___ai'] += 1

        d_worker = Worker(companyMap[d_comp], int(d_bonus), False, int(d_skillcount))
        for d_skill in d_skills:
            d_worker.add_raw_skill(d_skill)
        workers.append(d_worker)
        arguments['currentDevs'] += 1
        if arguments['currentDevs'] >= arguments['devCount']:
            new_status = 4
        else:
            new_status = 3
    elif status == 4:
        # Manager Count

        arguments.update({
            'pmCount': int(line),
            'currentPMs': 0
        })
        new_status = 5
    elif status == 5:
        # Manager
        m_comp, m_bonus = line.split(' ')
        if m_comp not in companyMap:
            companyMap[m_comp] = companyMap['___ai']
            companyMap['___ai'] += 1
        m_worker = Worker(companyMap[m_comp], int(m_bonus), False, 0)
        workers.append(m_worker)
        arguments['currentPMs'] += 1
        if arguments['currentPMs'] >= arguments['pmCount']:
            new_status = 6
        else:
            new_status = 5
    elif status == 6:
        raise Exception('We don\'t do status 6')
    return new_status


layout = nx.MultiGraph(x=0, y=0, m=0)
workers = []
companyMap = {
    '___ai': 0
}
read_input()
print(layout.nodes)
print(layout.edges)
for worker in workers:
    print(worker)

