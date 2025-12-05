// @ts-check
// Note: type annotations allow type checking and IDE autocompletion

const {themes} = require('prism-react-renderer');
const lightCodeTheme = themes.github;
const darkCodeTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  // Site Metadata
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Textbook for Teaching Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Deployment
  url: 'https://yo-its-anas.github.io',
  baseUrl: '/Phyiscal_AI_Humanoid_Robotics_Book/',
  organizationName: 'yo-its-anas',
  projectName: 'Phyiscal_AI_Humanoid_Robotics_Book',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  // Build Configuration
  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Internationalization
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Presets
  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: '/',
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/yo-its-anas/Phyiscal_AI_Humanoid_Robotics_Book/tree/main/',
          showLastUpdateTime: false,
          showLastUpdateAuthor: false,
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  // Theme Configuration
  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Navbar
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/yo-its-anas/Phyiscal_AI_Humanoid_Robotics_Book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },

      // Footer
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Content',
            items: [
              {
                label: 'Preface',
                to: '/preface',
              },
              {
                label: 'Part I: Foundations',
                to: '/physical-ai',
              },
              {
                label: 'Appendices',
                to: '/appendix',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/',
              },
              {
                label: 'NVIDIA Isaac Sim',
                href: 'https://docs.omniverse.nvidia.com/isaacsim/',
              },
              {
                label: 'Gazebo',
                href: 'https://gazebosim.org/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/yo-its-anas/Phyiscal_AI_Humanoid_Robotics_Book',
              },
              {
                label: 'Created with Claude Code',
                href: 'https://claude.com/claude-code',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
      },

      // Syntax Highlighting
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'bash', 'yaml', 'json', 'cpp', 'markup'],
      },

      // Table of Contents
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 4,
      },
    }),

  // Plugins
  plugins: [
    // Local search plugin (alternative to Algolia)
    [
      require.resolve('@cmfcmf/docusaurus-search-local'),
      {
        indexDocs: true,
        indexBlog: false,
        language: 'en',
      },
    ],
  ],

  // Markdown Configuration
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],
};

module.exports = config;
