# Contract: Docusaurus Configuration

**Feature**: 001-ai-robotics-textbook
**Date**: 2025-12-05
**Phase**: 1 - Design & Contracts

## Purpose

This contract defines the configuration requirements for the Docusaurus static site, ensuring consistent behavior, branding, and deployment.

## Configuration File: `docusaurus.config.js`

### Required Configuration

```javascript
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
  url: 'https://<username>.github.io',
  baseUrl: '/ai_robotics_book/',
  organizationName: '<github-username>',
  projectName: 'ai_robotics_book',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  // Build Configuration
  onBrokenLinks: 'throw',
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
          editUrl: 'https://github.com/<username>/ai_robotics_book/tree/main/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
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
            href: 'https://github.com/<username>/ai_robotics_book',
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
                to: '/01-physical-ai',
              },
              {
                label: 'Appendices',
                to: '/08-appendix',
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
                href: 'https://github.com/<username>/ai_robotics_book',
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
        additionalLanguages: ['python', 'bash', 'yaml', 'json', 'cpp', 'xml'],
      },

      // Table of Contents
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 4,
      },

      // Algolia Search (optional, configure later)
      // algolia: {
      //   appId: 'YOUR_APP_ID',
      //   apiKey: 'YOUR_SEARCH_API_KEY',
      //   indexName: 'ai_robotics',
      // },
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
```

## Configuration Requirements

### 1. Site Metadata

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `title` | string | Yes | Site title displayed in browser tab |
| `tagline` | string | Yes | Subtitle/description |
| `favicon` | string | Yes | Path to favicon in `static/` |
| `url` | string | Yes | Production URL |
| `baseUrl` | string | Yes | Base path for GitHub Pages |

**Validation**:
- `url` must be valid HTTPS URL
- `baseUrl` must match repository name for GitHub Pages
- `favicon` file must exist in `static/img/`

### 2. Deployment Configuration

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `organizationName` | string | Yes | GitHub username or org |
| `projectName` | string | Yes | Repository name |
| `deploymentBranch` | string | Yes | Branch for GitHub Pages (typically `gh-pages`) |
| `trailingSlash` | boolean | Yes | Set to `false` for GitHub Pages |

**Validation**:
- Must match actual GitHub repository details
- `deploymentBranch` should not be the main development branch

### 3. Build Configuration

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `onBrokenLinks` | string | Yes | Set to `'throw'` to fail builds on broken links |
| `onBrokenMarkdownLinks` | string | Yes | Set to `'warn'` or `'throw'` |

**Validation**:
- `onBrokenLinks: 'throw'` enforces link quality
- Prevents publishing broken documentation

### 4. Documentation Settings

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `routeBasePath` | string | Yes | Set to `'/'` to make docs the homepage |
| `sidebarPath` | string | Yes | Path to `sidebars.js` |
| `editUrl` | string | Yes | GitHub edit link for each page |
| `showLastUpdateTime` | boolean | Yes | Display last modified date |
| `showLastUpdateAuthor` | boolean | Yes | Display last author (from Git) |

**Validation**:
- `editUrl` must point to correct GitHub repository
- Enables "Edit this page" links for contributors

### 5. Theme Configuration

#### Navbar

- **Logo**: Must be SVG or PNG, located in `static/img/`
- **Navigation Items**: Links to main sections
- **GitHub Link**: Required in top-right corner

#### Footer

- **Links**: Organized into 3 columns
  - Content navigation
  - External resources
  - Meta (GitHub, credits)
- **Copyright**: Auto-updates year

#### Syntax Highlighting

**Required Languages**:
- Python (ROS 2 nodes)
- Bash (scripts)
- YAML (config files)
- JSON (data)
- C++ (advanced examples)
- XML (URDF, launch files)

### 6. Plugins

#### Search Plugin

**Option 1: Local Search** (recommended for start)
```javascript
'@cmfcmf/docusaurus-search-local'
```

**Option 2: Algolia DocSearch** (for production)
- Free for open-source projects
- Requires application and approval
- Better search quality

**Validation**:
- At least one search solution must be configured
- Local search works immediately, Algolia requires setup

#### Mermaid Diagrams

**Required**:
```javascript
markdown: {
  mermaid: true,
},
themes: ['@docusaurus/theme-mermaid'],
```

**Purpose**: Enables inline diagram creation with Mermaid syntax

### 7. Performance Requirements

| Metric | Target | Validation Method |
|--------|--------|-------------------|
| Build Time | < 30 seconds | `npm run build` timing |
| Page Load | < 2 seconds | Lighthouse audit |
| Lighthouse Score | > 90 | Automated CI check |
| Bundle Size | < 500KB | Webpack bundle analyzer |

### 8. Accessibility Requirements

| Requirement | Implementation | Validation |
|-------------|----------------|------------|
| WCAG 2.1 AA | Theme defaults | Lighthouse accessibility audit |
| Keyboard Navigation | Native Docusaurus support | Manual testing |
| Screen Reader Support | Semantic HTML, alt text | NVDA/VoiceOver testing |
| Color Contrast | Theme ensures 4.5:1 minimum | Contrast checker |

## Package Dependencies

### Required Packages (`package.json`)

```json
{
  "name": "ai-robotics-textbook",
  "version": "1.0.0",
  "private": true,
  "scripts": {
    "docusaurus": "docusaurus",
    "start": "docusaurus start",
    "build": "docusaurus build",
    "swizzle": "docusaurus swizzle",
    "deploy": "docusaurus deploy",
    "clear": "docusaurus clear",
    "serve": "docusaurus serve",
    "write-translations": "docusaurus write-translations",
    "write-heading-ids": "docusaurus write-heading-ids"
  },
  "dependencies": {
    "@docusaurus/core": "^3.0.0",
    "@docusaurus/preset-classic": "^3.0.0",
    "@docusaurus/theme-mermaid": "^3.0.0",
    "@mdx-js/react": "^3.0.0",
    "clsx": "^2.0.0",
    "prism-react-renderer": "^2.1.0",
    "react": "^18.0.0",
    "react-dom": "^18.0.0"
  },
  "devDependencies": {
    "@cmfcmf/docusaurus-search-local": "^1.1.0",
    "@docusaurus/module-type-aliases": "^3.0.0",
    "@docusaurus/types": "^3.0.0"
  },
  "engines": {
    "node": ">=18.0"
  }
}
```

**Validation**:
- Node.js 18+ required
- All peer dependencies must be satisfied
- Lock file (`package-lock.json`) must be committed

## File Locations

```
ai_robotics_book/
├── docusaurus.config.js    # Main config (this contract)
├── package.json            # Dependencies
├── sidebars.js            # Navigation structure (separate contract)
├── src/
│   └── css/
│       └── custom.css     # Theme customization
└── static/
    └── img/
        ├── logo.svg       # Site logo
        └── favicon.ico    # Browser icon
```

## Customization: `src/css/custom.css`

```css
/* Custom CSS variables */
:root {
  --ifm-color-primary: #2e8555;
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
  --ifm-color-primary-darkest: #205d3b;
  --ifm-color-primary-light: #33925d;
  --ifm-color-primary-lighter: #359962;
  --ifm-color-primary-lightest: #3cad6e;
  --ifm-code-font-size: 95%;
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.1);
}

/* Dark mode */
[data-theme='dark'] {
  --ifm-color-primary: #25c2a0;
  --ifm-color-primary-dark: #21af90;
  --ifm-color-primary-darker: #1fa588;
  --ifm-color-primary-darkest: #1a8870;
  --ifm-color-primary-light: #29d5b0;
  --ifm-color-primary-lighter: #32d8b4;
  --ifm-color-primary-lightest: #4fddbf;
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.3);
}

/* Custom typography for robotics content */
.markdown h1 {
  font-size: 2.5rem;
  margin-top: 0;
}

.markdown h2 {
  font-size: 2rem;
  margin-top: 3rem;
  margin-bottom: 1rem;
  border-bottom: 1px solid var(--ifm-color-emphasis-300);
  padding-bottom: 0.5rem;
}

/* Code block enhancements */
.prism-code {
  font-size: 0.9rem;
  line-height: 1.5;
}

/* Admonitions for important notes */
.admonition {
  margin-bottom: 1rem;
}
```

## Environment Variables

For GitHub Actions deployment:

```yaml
# .github/workflows/deploy.yml
env:
  NODE_VERSION: '18'
  DOCUSAURUS_URL: 'https://username.github.io'
  DOCUSAURUS_BASE_URL: '/ai_robotics_book/'
```

## Validation Checklist

Before deployment:

- [ ] `npm install` completes without errors
- [ ] `npm run build` succeeds
- [ ] `npm run serve` shows correct site locally
- [ ] All navigation links work
- [ ] Search functionality works
- [ ] Images and diagrams load correctly
- [ ] Code syntax highlighting displays properly
- [ ] Mobile responsive (test on phone)
- [ ] Lighthouse score > 90
- [ ] No broken links or markdown errors
- [ ] Edit links point to correct GitHub URLs
- [ ] Footer links are accurate

## Migration Path

### From Development to Production

1. **Local Development**:
   ```bash
   npm start
   # Live reload at http://localhost:3000
   ```

2. **Build Verification**:
   ```bash
   npm run build
   npm run serve
   # Test production build at http://localhost:3000
   ```

3. **Deployment**:
   ```bash
   # Automated via GitHub Actions
   # Or manual: npm run deploy
   ```

### Updating Configuration

When updating `docusaurus.config.js`:

1. Test locally first
2. Verify build succeeds
3. Check no regressions in navigation
4. Commit changes
5. Deploy via CI/CD

## Non-Functional Requirements

### Performance

- **Build time**: < 30 seconds for full site
- **Hot reload**: < 1 second for content changes
- **Page load**: < 2 seconds on 3G connection

### Reliability

- **Link validation**: 100% internal links must work
- **Build success rate**: > 99% (only fail on code errors, not transient issues)
- **Uptime**: GitHub Pages SLA (99.9%+)

### Scalability

- **Content volume**: Support 300+ pages
- **Code examples**: 100+ downloadable samples
- **Images**: Optimize all images < 200KB each
- **Search index**: Handle full textbook content

## References

- [Docusaurus Configuration](https://docusaurus.io/docs/configuration)
- [GitHub Pages Deployment](https://docusaurus.io/docs/deployment#deploying-to-github-pages)
- [Docusaurus Plugins](https://docusaurus.io/docs/using-plugins)
- [Mermaid Diagrams](https://docusaurus.io/docs/markdown-features/diagrams)
