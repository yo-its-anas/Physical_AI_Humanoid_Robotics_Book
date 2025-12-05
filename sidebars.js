/**
 * Sidebar configuration for Physical AI & Humanoid Robotics Textbook
 *
 * Structure:
 * - Preface (5 pages)
 * - 8 Parts (27 chapters total + part intros)
 * - Appendices (5 reference documents)
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    // ========================================
    // PREFACE
    // ========================================
    {
      type: 'category',
      label: 'Preface',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'preface/index',
          label: 'What This Textbook Is',
        },
        {
          type: 'doc',
          id: 'preface/how-created',
          label: 'How This Book Was Created',
        },
        {
          type: 'doc',
          id: 'preface/audience',
          label: 'Who This Book Is For',
        },
        {
          type: 'doc',
          id: 'preface/how-to-use',
          label: 'How to Use This Book',
        },
        {
          type: 'doc',
          id: 'preface/requirements',
          label: 'Requirements Overview',
        },
      ],
    },

    // ========================================
    // PART I: FOUNDATIONS
    // ========================================
    {
      type: 'category',
      label: 'Part I: Foundations of Physical AI',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'physical-ai/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: 'physical-ai/01-introduction',
          label: 'Ch 1: Introduction to Physical AI',
        },
        {
          type: 'doc',
          id: 'physical-ai/02-humanoid-landscape',
          label: 'Ch 2: Humanoid Robotics Landscape',
        },
        {
          type: 'doc',
          id: 'physical-ai/03-sensor-foundations',
          label: 'Ch 3: Sensor Foundations',
        },
        {
          type: 'doc',
          id: 'physical-ai/04-weekly-overview',
          label: 'Ch 4: Course Overview',
        },
      ],
    },

    // ========================================
    // PART II: ROS 2
    // ========================================
    {
      type: 'category',
      label: 'Part II: ROS 2',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'ros2/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: 'ros2/05-fundamentals',
          label: 'Ch 5: ROS 2 Fundamentals',
        },
        {
          type: 'doc',
          id: 'ros2/06-python-development',
          label: 'Ch 6: Python Development',
        },
        {
          type: 'doc',
          id: 'ros2/07-urdf',
          label: 'Ch 7: URDF Robot Description',
        },
        {
          type: 'doc',
          id: 'ros2/08-control-systems',
          label: 'Ch 8: Control Systems',
        },
      ],
    },

    // ========================================
    // PART III: SIMULATION
    // ========================================
    {
      type: 'category',
      label: 'Part III: Gazebo & Unity',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'gazebo-unity/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: 'gazebo-unity/09-gazebo-intro',
          label: 'Ch 9: Gazebo Simulation',
        },
        {
          type: 'doc',
          id: 'gazebo-unity/10-humanoid-sim',
          label: 'Ch 10: Simulating Humanoids',
        },
        {
          type: 'doc',
          id: 'gazebo-unity/11-unity-robotics',
          label: 'Ch 11: Unity for Robotics',
        },
      ],
    },

    // ========================================
    // PART IV: NVIDIA ISAAC
    // ========================================
    {
      type: 'category',
      label: 'Part IV: NVIDIA Isaac',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'nvidia-isaac/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: 'nvidia-isaac/12-isaac-sim',
          label: 'Ch 12: Isaac Sim Introduction',
        },
        {
          type: 'doc',
          id: 'nvidia-isaac/13-perception',
          label: 'Ch 13: Perception & Synthetic Data',
        },
        {
          type: 'doc',
          id: 'nvidia-isaac/14-isaac-ros',
          label: 'Ch 14: Isaac ROS',
        },
        {
          type: 'doc',
          id: 'nvidia-isaac/15-navigation',
          label: 'Ch 15: Navigation (Nav2)',
        },
      ],
    },

    // ========================================
    // PART V: VISION-LANGUAGE-ACTION
    // ========================================
    {
      type: 'category',
      label: 'Part V: Vision-Language-Action',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'vla/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: 'vla/16-vla-intro',
          label: 'Ch 16: VLA Systems Introduction',
        },
        {
          type: 'doc',
          id: 'vla/17-voice-whisper',
          label: 'Ch 17: Voice with Whisper',
        },
        {
          type: 'doc',
          id: 'vla/18-llm-planning',
          label: 'Ch 18: LLM Planning',
        },
        {
          type: 'doc',
          id: 'vla/19-multimodal',
          label: 'Ch 19: Multimodal Interaction',
        },
      ],
    },

    // ========================================
    // PART VI: HARDWARE LAB
    // ========================================
    {
      type: 'category',
      label: 'Part VI: Hardware Lab',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'hardware-lab/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: 'hardware-lab/20-workstations',
          label: 'Ch 20: Simulation Workstations',
        },
        {
          type: 'doc',
          id: 'hardware-lab/21-jetson',
          label: 'Ch 21: Jetson Edge AI Kits',
        },
        {
          type: 'doc',
          id: 'hardware-lab/22-robot-options',
          label: 'Ch 22: Robot Platform Options',
        },
        {
          type: 'doc',
          id: 'hardware-lab/23-cloud-infra',
          label: 'Ch 23: Cloud Infrastructure',
        },
      ],
    },

    // ========================================
    // PART VII: CAPSTONE
    // ========================================
    {
      type: 'category',
      label: 'Part VII: Capstone Project',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'capstone/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: 'capstone/24-architecture',
          label: 'Ch 24: System Architecture',
        },
        {
          type: 'doc',
          id: 'capstone/25-implementation',
          label: 'Ch 25: Implementation',
        },
        {
          type: 'doc',
          id: 'capstone/26-sim-to-real',
          label: 'Ch 26: Sim-to-Real Transfer',
        },
        {
          type: 'doc',
          id: 'capstone/27-evaluation',
          label: 'Ch 27: Evaluation & Extensions',
        },
      ],
    },

    // ========================================
    // PART VIII: APPENDICES
    // ========================================
    {
      type: 'category',
      label: 'Appendices',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'appendix/index',
          label: 'Appendices Overview',
        },
        {
          type: 'doc',
          id: 'appendix/a-ros2-cheatsheet',
          label: 'A: ROS 2 Cheat Sheets',
        },
        {
          type: 'doc',
          id: 'appendix/b-urdf-reference',
          label: 'B: URDF/SDF Reference',
        },
        {
          type: 'doc',
          id: 'appendix/c-troubleshooting-guide',
          label: 'C: Troubleshooting Guide',
        },
        {
          type: 'doc',
          id: 'appendix/d-hardware-checklists',
          label: 'D: Hardware Checklists',
        },
        {
          type: 'doc',
          id: 'appendix/e-instructor-guides',
          label: 'E: Instructor Guides',
        },
      ],
    },
  ],
};

module.exports = sidebars;
